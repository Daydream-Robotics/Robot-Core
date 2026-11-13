#include "perception/lidar/lidarDriver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <utility>

#ifdef __jetson_nano__
#include <fcntl.h>
#include <linux/serial.h>
#include <pthread.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace {
constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;
constexpr double INCHES_PER_MM = 1.0 / 25.4;
constexpr size_t MAX_SAMPLES = 4096;
// Sync times are fitted over this many recent sweeps (~1.6 s at 10 Hz).
constexpr size_t SYNC_FIT_WINDOW = 16;
// The fitted sample period must stay this close to the scan mode's us_per_sample.
constexpr double MAX_SLOPE_ERROR = 0.02;
// A sync this much later than the fit is a read delay once, or dropped samples twice in a row.
constexpr double SYNC_RESYNC_US = 3000.0;

uint64_t nowUs() {
#ifdef _WIN32
    // std::chrono::steady_clock uses a monotonic clock on Windows (normally QPC).
    // Its epoch is local to this process/platform; do not compare with a Jetson
    // timestamp until both systems are explicitly synchronized.
    return static_cast<uint64_t>(std::chrono::duration_cast<
        std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
#else
    // Same clock as the Jetson's other CLOCK_MONOTONIC timestamps (issue #139).
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL +
           static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
#endif
}

void sleepInterruptibly(const std::atomic<bool>& running, uint64_t us) {
    for (uint64_t slept = 0; running && slept < us; slept += 10000)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

// Normalize into [-pi, pi]. 2pi and 0 correspond to the same direction.
double signedBearing(double angleCw) {
    double rad = std::remainder(-angleCw, TWO_PI);
    return rad >= PI ? rad - TWO_PI : rad;
}

// Travel distance in clockwise angle from the start of this revolution.
double sweepAngle(double current, double start) {
    double d = std::fmod(current - start + TWO_PI, TWO_PI);
    return d < 0.0 ? d + TWO_PI : d;
}
} // namespace

LidarDriver::LidarDriver(LidarConfig cfg) : m_cfg(std::move(cfg)) {
    m_health.configuredLatencyUs = m_cfg.latencyUs;
    m_building.reserve(1024);
}

LidarDriver::~LidarDriver() { stop(); }

bool LidarDriver::start(ScanCallback onScan) {
    if (!onScan || m_cfg.device.empty() || m_cfg.baud <= 0 ||
        m_cfg.timeoutUs < 250000 || m_cfg.minPoints == 0 ||
        m_cfg.startupGraceUs < m_cfg.timeoutUs || m_cfg.retryBackoffUs == 0 ||
        m_cfg.readThreadPriority < 0 || m_cfg.readThreadPriority > 99 ||
        m_cfg.emptyReadSleepUs < 100 || m_cfg.emptyReadSleepUs > 10000) return false;
    if (m_running.exchange(true)) return false;
    m_onScan = std::move(onScan);
    try {
        m_thread = std::thread(&LidarDriver::run, this);
#ifdef __jetson_nano__
        if (m_cfg.readThreadPriority > 0) {
            // Same mechanism as SerialLink::makeRealtime (reader=80, writer=81).
            // Lack of CAP_SYS_NICE is nonfatal; continue under normal scheduling.
            sched_param param{};
            param.sched_priority = m_cfg.readThreadPriority;
            const int error = pthread_setschedparam(
                m_thread.native_handle(), SCHED_FIFO, &param);
            if (error != 0) {
                std::fprintf(stderr,
                    "[LiDAR %u] SCHED_FIFO priority %d unavailable (error=%d); using normal priority\n",
                    unsigned(m_cfg.sensorId), m_cfg.readThreadPriority, error);
            }
        }
#endif
    } catch (...) {
        m_running = false;
        return false;
    }
    return true;
}

void LidarDriver::stop() {
    m_running = false;
    if (m_thread.joinable()) m_thread.join();
    // The worker alone owns the SDK objects. Join must precede cleanup.
}

LidarHealth LidarDriver::health() const {
    std::lock_guard<std::mutex> lock(m_healthMutex);
    return m_health;
}

void LidarDriver::resetSweepState() {
    m_building.clear();
    m_buildingOverflow = false;
    m_tStartUs = 0;
    m_sampleIndex = 0;
    m_syncObs.clear();
    m_lateSyncs = 0;
    m_lastPeriodUs = 0;
    m_startAngleCwRad = 0.0;
}

bool LidarDriver::connectAndStart() {
    resetSweepState();
    std::string device = m_cfg.device;
#ifdef _WIN32
    // Windows ports above COM9 need the Win32 extended device prefix.
    // Accept familiar CLion command-line args like COM15 as well as \.\COM15.
    if ((device.rfind("COM", 0) == 0 || device.rfind("com", 0) == 0) &&
        device.rfind("\\\\.\\", 0) != 0) {
        device = "\\\\.\\" + device;
    }
#endif
    auto channelResult = sl::createSerialPortChannel(device, m_cfg.baud);
    if (!channelResult || !*channelResult) {
        std::fprintf(stderr, "[LiDAR %u] createSerialPortChannel failed: %s (%d baud)\n",
                     unsigned(m_cfg.sensorId), device.c_str(), m_cfg.baud);
        return false;
    }
    m_channel = *channelResult;

    auto driverResult = sl::createLidarDriver();
    if (!driverResult || !*driverResult) {
        std::fprintf(stderr, "[LiDAR %u] createLidarDriver failed\n", unsigned(m_cfg.sensorId));
        shutdownConnection(false);
        return false;
    }
    m_drv = *driverResult;

    const sl_result connectionResult = m_drv->connect(m_channel);
    if (SL_IS_FAIL(connectionResult)) {
        std::fprintf(stderr, "[LiDAR %u] connect failed: 0x%08x\n",
                     unsigned(m_cfg.sensorId), unsigned(connectionResult));
        shutdownConnection(false);
        return false;
    }
    sl_lidar_response_device_health_t devHealth{};
    const sl_result healthResult = m_drv->getHealth(devHealth);
    if (SL_IS_FAIL(healthResult) || devHealth.status == SL_LIDAR_STATUS_ERROR) {
        std::fprintf(stderr, "[LiDAR %u] getHealth failed: result=0x%08x status=%u\n",
                     unsigned(m_cfg.sensorId), unsigned(healthResult), unsigned(devHealth.status));
        shutdownConnection(false);
        return false;
    }

    const sl_result motorResult = m_drv->setMotorSpeed();
    if (SL_IS_FAIL(motorResult)) {
        std::fprintf(stderr, "[LiDAR %u] setMotorSpeed failed: 0x%08x\n",
                     unsigned(m_cfg.sensorId), unsigned(motorResult));
        shutdownConnection(false);
        return false;
    }
    sl::LidarScanMode scanMode{};
    const sl_result scanResult = m_drv->startScan(false, true, 0, &scanMode);
    if (SL_IS_FAIL(scanResult) || scanMode.us_per_sample <= 0.0f) {
        std::fprintf(stderr, "[LiDAR %u] startScan failed: result=0x%08x us_per_sample=%.1f\n",
                     unsigned(m_cfg.sensorId), unsigned(scanResult), double(scanMode.us_per_sample));
        shutdownConnection(false);
        return false;
    }

    m_usPerSample = scanMode.us_per_sample;
    m_lastChecksumErrors = 0; // new SDK driver, its count starts at zero
    m_lastDataRxUs = m_lastSyncRxUs = nowUs();
    m_connected = true;
    openUartCounters();
    std::fprintf(stderr, "[LiDAR %u] scan started: port=%s baud=%d mode=%s us_per_sample=%.1f "
                         "(allowing %.1f s for first samples)\n",
                 unsigned(m_cfg.sensorId), device.c_str(), m_cfg.baud, scanMode.scan_mode,
                 double(m_usPerSample), double(m_cfg.startupGraceUs) / 1000000.0);
    return true;
}

void LidarDriver::shutdownConnection(bool reset) {
    pollUartCounters();
    pollChecksumErrors();
    closeUartCounters();
    m_connected = false;
    if (m_channel && m_drv && m_drv->isConnected())
        m_channel->clearReadCache(); // flush buffered incoming bytes
    if (m_drv) {
        // These calls can fail on a physically disconnected sensor; still clean up.
        if (m_drv->isConnected()) {
            const auto scanStop = m_drv->stop(300);
            const auto motorStop = m_drv->setMotorSpeed(0);

            std::fprintf(stderr,
                "[LiDAR %u] shutdown: scanStop=0x%08x motorStop=0x%08x\n",
                unsigned(m_cfg.sensorId),
                unsigned(scanStop),
                unsigned(motorStop));

            if (reset) {
                m_drv->reset(300);
            }

            m_drv->disconnect();
        }
        delete m_drv;
        m_drv = nullptr;
    }
    delete m_channel;
    m_channel = nullptr;
    resetSweepState();
}

void LidarDriver::recover(bool wasStreaming) {
    shutdownConnection(true);
    if (m_running && wasStreaming) {
        std::lock_guard<std::mutex> lock(m_healthMutex);
        ++m_health.restarts;
    }
}

void LidarDriver::run() {
    // Non-blocking SDK interval reads, one driver and one thread per sensor.
    sl_lidar_response_measurement_node_hq_t nodes[2048];
    uint64_t startedUs = 0;
    bool reportedData = false;
    bool reportedSync = false;
    while (m_running) {
        if (!m_drv) {
            if (!connectAndStart()) {
                // Failed initial/retry connects are not stream recoveries.
                sleepInterruptibly(m_running, m_cfg.retryBackoffUs);
                continue;
            }
            startedUs = nowUs();
            reportedData = false;
            reportedSync = false;
        }
        size_t count = sizeof(nodes) / sizeof(nodes[0]);
        const sl_result result = m_drv->getScanDataWithIntervalHq(nodes, count);
        const uint64_t rxUs = nowUs(); // never use time of full-sweep grab
        // Poll health occasionally rather than performing an ioctl per node.
        if (rxUs - m_lastUartPollUs >= 1000000ULL) {
            pollUartCounters();
            pollChecksumErrors();
            m_lastUartPollUs = rxUs;
        }

        if (SL_IS_OK(result) && count > 0) {
            if (!reportedData) {
                std::fprintf(stderr, "[LiDAR %u] first SDK batch: %zu nodes, %llu ms after scan start\n",
                             unsigned(m_cfg.sensorId), count,
                             static_cast<unsigned long long>((rxUs - startedUs) / 1000));
                reportedData = true;
            }
            m_lastDataRxUs = rxUs;
            onSamples(nodes, count, rxUs);
            if (m_tStartUs && !reportedSync) {
                std::fprintf(stderr, "[LiDAR %u] first revolution sync observed\n",
                             unsigned(m_cfg.sensorId));
                reportedSync = true;
            }
        } else if (result != SL_RESULT_OPERATION_TIMEOUT && SL_IS_FAIL(result)) {
            std::fprintf(stderr, "[LiDAR %u] interval SDK read failed: 0x%08x\n",
                         unsigned(m_cfg.sensorId), unsigned(result));
            {
                std::lock_guard<std::mutex> lock(m_healthMutex);
                ++m_health.readErrors;
            }
            recover(reportedData && reportedSync);
            sleepInterruptibly(m_running, m_cfg.retryBackoffUs);
            continue;
        } else {
            // getScanDataWithIntervalHq() can return SL_RESULT_OK with count=0.
            std::this_thread::sleep_for(std::chrono::microseconds(m_cfg.emptyReadSleepUs));
        }

        // A working C1 may take longer than 500 ms to spin up. Wait for a
        // bounded startup grace period before judging the stream's health.
        if (rxUs - startedUs < m_cfg.startupGraceUs) continue;

        const bool noData = rxUs - m_lastDataRxUs > m_cfg.timeoutUs;
        const bool noSync = !m_tStartUs || rxUs - m_lastSyncRxUs > m_cfg.timeoutUs;
        if (noData || noSync) {
            std::fprintf(stderr,
                         "[LiDAR %u] stream timeout: noData=%d noSync=%d "
                         "msSinceData=%llu msSinceSync=%llu -> reconnecting\n",
                         unsigned(m_cfg.sensorId), int(noData), int(noSync),
                         static_cast<unsigned long long>((rxUs - m_lastDataRxUs) / 1000),
                         static_cast<unsigned long long>((rxUs - m_lastSyncRxUs) / 1000));
            {
                std::lock_guard<std::mutex> lock(m_healthMutex);
                ++m_health.syncLoss;
                if (m_tStartUs) ++m_health.droppedSweeps;
            }
            recover(reportedData && reportedSync);
            sleepInterruptibly(m_running, m_cfg.retryBackoffUs);
        }
    }
    shutdownConnection(false);
}

void LidarDriver::onSamples(const sl_lidar_response_measurement_node_hq_t* nodes,
                            size_t count, uint64_t rxUs) {
    // The SDK returns nodes in acquisition order. Nodes earlier in the batch
    // arrived earlier, one sample period apart.
    for (size_t i = 0; i < count; ++i) {
        const uint64_t index = m_sampleIndex++;
        const auto& n = nodes[i];
        const double angleCw = static_cast<double>(n.angle_z_q14) *
                               (90.0 / 16384.0) * (PI / 180.0);
        if (n.flag & SL_LIDAR_RESP_HQ_FLAG_SYNCBIT) {
            const auto backUs = static_cast<uint64_t>(
                std::llround((count - 1 - i) * static_cast<double>(m_usPerSample)));
            const uint64_t rawUs = rxUs > backUs ? rxUs - backUs : 0;
            const uint64_t syncUs = fitSyncTime(index, rawUs);
            // The sync sample ends the previous revolution and begins the next.
            m_lastSyncRxUs = rxUs;
            if (m_tStartUs) finishSweep(syncUs);
            m_tStartUs = syncUs;
            m_startAngleCwRad = angleCw;
            m_building.clear();
            m_buildingOverflow = false;
        }
        if (!m_tStartUs) continue; // discard partial first revolution
        if (n.dist_mm_q2 == 0 || n.quality == 0) continue;
        if (m_building.size() >= MAX_SAMPLES) {
            m_buildingOverflow = true;
            continue;
        }
        m_building.push_back({angleCw,
            static_cast<double>(n.dist_mm_q2) * 0.25 * INCHES_PER_MM, n.quality});
    }
}

// rawUs is worked back from when its batch was read, so read delays make it late
// by a varying amount but never early. Fitting a line through the last few syncs
// (time vs sample index) and moving it down to the earliest one removes that
// jitter. The configured latency is then subtracted.
uint64_t LidarDriver::fitSyncTime(uint64_t index, uint64_t rawUs) {
    m_syncObs.push_back({index, rawUs});
    if (m_syncObs.size() > SYNC_FIT_WINDOW) m_syncObs.pop_front();

    const SyncObservation& first = m_syncObs.front();
    auto dIndex = [&](const SyncObservation& o) { return static_cast<double>(o.index - first.index); };
    auto dTime = [&](const SyncObservation& o) {
        return static_cast<double>(static_cast<int64_t>(o.rawUs - first.rawUs));
    };

    // Least-squares slope in us per sample, or the scan mode's value with too few syncs.
    double slope = m_usPerSample;
    if (m_syncObs.size() >= 3) {
        double meanX = 0.0, meanY = 0.0;
        for (const auto& o : m_syncObs) { meanX += dIndex(o); meanY += dTime(o); }
        meanX /= m_syncObs.size();
        meanY /= m_syncObs.size();
        double sxx = 0.0, sxy = 0.0;
        for (const auto& o : m_syncObs) {
            sxx += (dIndex(o) - meanX) * (dIndex(o) - meanX);
            sxy += (dIndex(o) - meanX) * (dTime(o) - meanY);
        }
        const double fitted = sxx > 0.0 ? sxy / sxx : slope;
        if (std::abs(fitted - m_usPerSample) <= MAX_SLOPE_ERROR * m_usPerSample) slope = fitted;
    }

    // Lower envelope: the earliest-looking sync had the least read delay.
    double minResidual = 0.0;
    for (const auto& o : m_syncObs)
        minResidual = std::min(minResidual, dTime(o) - slope * dIndex(o));
    const double newestResidual = dTime(m_syncObs.back()) - slope * dIndex(m_syncObs.back());

    double fittedUs = static_cast<double>(first.rawUs) + minResidual + slope * dIndex(m_syncObs.back());
    if (newestResidual - minResidual <= SYNC_RESYNC_US) {
        m_lateSyncs = 0;
    } else if (++m_lateSyncs < 2 && m_syncObs.size() > 1) {
        // One late sync is a long read delay, keep the fit and drop this observation.
        m_syncObs.pop_back();
    } else {
        // Late twice in a row means dropped samples shifted the index to time relation,
        // so restart the fit from this sync.
        m_syncObs.erase(m_syncObs.begin(), m_syncObs.end() - 1);
        m_lateSyncs = 0;
        fittedUs = static_cast<double>(rawUs);
    }
    const double stampUs = fittedUs - static_cast<double>(m_cfg.latencyUs);
    return stampUs > 0.0 ? static_cast<uint64_t>(std::llround(stampUs)) : 0;
}

void LidarDriver::finishSweep(uint64_t tEndUs) {
    if (tEndUs <= m_tStartUs) {
        std::lock_guard<std::mutex> lock(m_healthMutex);
        ++m_health.syncLoss;
        ++m_health.droppedSweeps;
        return;
    }
    const uint64_t periodUs = tEndUs - m_tStartUs;
    const double hz = 1000000.0 / static_cast<double>(periodUs);
    {
        std::lock_guard<std::mutex> lock(m_healthMutex);
        m_health.lastSweepJitterUs = m_lastPeriodUs ?
            (periodUs > m_lastPeriodUs ? periodUs - m_lastPeriodUs : m_lastPeriodUs - periodUs) : 0;
        m_health.lastSweepPeriodUs = periodUs;
        m_health.lastSweepHz = hz;
    }
    m_lastPeriodUs = periodUs;

    if (hz < 8.0 || hz > 12.0 || m_buildingOverflow ||
        m_building.size() < m_cfg.minPoints) {
        std::lock_guard<std::mutex> lock(m_healthMutex);
        ++m_health.droppedSweeps;
        return;
    }

    Scan scan;
    scan.sensor_id = m_cfg.sensorId;
    scan.t_start_us = m_tStartUs;
    scan.t_end_us = tEndUs;
    scan.points.reserve(m_building.size());
    uint64_t lastStamp = scan.t_start_us;
    for (const auto& n : m_building) {
        const double fraction = sweepAngle(n.angleCwRad, m_startAngleCwRad) / TWO_PI;
        uint64_t stamp = scan.t_start_us + static_cast<uint64_t>(fraction * periodUs);
        // Guarantee non-decreasing timestamps even if angles jitter in SDK data.
        stamp = std::max(stamp, lastStamp);
        stamp = std::min(stamp, scan.t_end_us - 1);
        lastStamp = stamp;
        scan.points.push_back({n.rangeIn, signedBearing(n.angleCwRad), stamp, n.quality});
    }

    {
        std::lock_guard<std::mutex> lock(m_healthMutex);
        ++m_health.sweeps;
    }
    // Callback runs on this sensor's thread. Keep it fast, e.g. push a queue.
    // Do not lock the other sensor or perform expensive processing here.
    try {
        m_onScan(scan);
    } catch (...) {
        std::lock_guard<std::mutex> lock(m_healthMutex);
        ++m_health.callbackErrors;
    }
}

// The SDK owns its serial fd. This optional second fd is used for diagnostics
// only. Some USB serial adapters / drivers do not implement TIOCGICOUNT.
void LidarDriver::openUartCounters() {
#ifdef __jetson_nano__
    closeUartCounters();
    m_uartCountersFd = ::open(m_cfg.device.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (m_uartCountersFd >= 0) {
        struct serial_icounter_struct counters{};
        if (::ioctl(m_uartCountersFd, TIOCGICOUNT, &counters) == 0) {
            m_lastFrame = static_cast<uint32_t>(counters.frame);
            m_lastParity = static_cast<uint32_t>(counters.parity);
            m_lastOverrun = static_cast<uint32_t>(counters.overrun);
            m_lastBufferOverrun = static_cast<uint32_t>(counters.buf_overrun);
            m_lastUartPollUs = nowUs();
            std::lock_guard<std::mutex> lock(m_healthMutex);
            m_health.uartCountersAvailable = true;
            return;
        }
        ::close(m_uartCountersFd);
        m_uartCountersFd = -1;
    }
    if (!m_reportedUartCountersUnavailable) {
        std::fprintf(stderr,
            "[LiDAR %u] UART TIOCGICOUNT counters unavailable for %s; "
            "framing/parity/overrun counts cannot be measured\n",
            unsigned(m_cfg.sensorId), m_cfg.device.c_str());
        m_reportedUartCountersUnavailable = true;
    }
#endif
    std::lock_guard<std::mutex> lock(m_healthMutex);
    m_health.uartCountersAvailable = false;
}

void LidarDriver::pollUartCounters() {
#ifdef __jetson_nano__
    if (m_uartCountersFd < 0) return;
    struct serial_icounter_struct counters{};
    if (::ioctl(m_uartCountersFd, TIOCGICOUNT, &counters) != 0) {
        closeUartCounters();
        std::lock_guard<std::mutex> lock(m_healthMutex);
        m_health.uartCountersAvailable = false;
        return;
    }
    const auto frame = static_cast<uint32_t>(counters.frame);
    const auto parity = static_cast<uint32_t>(counters.parity);
    const auto overrun = static_cast<uint32_t>(counters.overrun);
    const auto bufferOverrun = static_cast<uint32_t>(counters.buf_overrun);
    // Unsigned subtraction also handles ordinary 32-bit counter wraparound.
    const uint32_t newFrames = frame - m_lastFrame;
    const uint32_t newParity = parity - m_lastParity;
    const uint32_t newOverruns = overrun - m_lastOverrun;
    const uint32_t newBufferOverruns = bufferOverrun - m_lastBufferOverrun;
    m_lastFrame = frame;
    m_lastParity = parity;
    m_lastOverrun = overrun;
    m_lastBufferOverrun = bufferOverrun;
    std::lock_guard<std::mutex> lock(m_healthMutex);
    m_health.framingErrors += newFrames;
    m_health.parityErrors += newParity;
    m_health.overrunErrors += newOverruns;
    m_health.bufferOverrunErrors += newBufferOverruns;
    m_health.readErrors += static_cast<uint64_t>(newFrames) + newParity +
                           newOverruns + newBufferOverruns;
#endif
}

// Packets the SDK dropped for a bad checksum. Needs the counter added by the SDK patch.
void LidarDriver::pollChecksumErrors() {
    if (!m_drv) return;
    const uint64_t total = m_drv->getChecksumErrorCount();
    const uint64_t added = total - m_lastChecksumErrors;
    m_lastChecksumErrors = total;
    if (added == 0) return;
    std::lock_guard<std::mutex> lock(m_healthMutex);
    m_health.checksumErrors += added;
    m_health.readErrors += added;
}

void LidarDriver::closeUartCounters() {
#ifdef __jetson_nano__
    if (m_uartCountersFd >= 0) {
        ::close(m_uartCountersFd);
        m_uartCountersFd = -1;
    }
#endif
    // Retain the last stream's availability for the final test report.
    // openUartCounters() will update it on the next connection.
}
