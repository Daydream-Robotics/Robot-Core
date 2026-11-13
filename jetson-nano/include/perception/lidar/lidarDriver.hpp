#pragma once

// SDK-based driver for one physical Slamtec sensor. Windows PC or Linux/Jetson Nano.
// This header deliberately follows the interface from Robot-Core issue #137.
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct LidarPoint {
    double range_in = 0.0;    // inches
    double bearing_rad = 0.0; // sensor frame, counterclockwise positive, [-pi, pi]
    uint64_t stamp_us = 0;    // monotonic microseconds (CLOCK_MONOTONIC on Linux)
    uint8_t quality = 0;      // optional, keeps diagnostic strength from SDK
};

struct Scan {
    uint8_t sensor_id = 0;
    uint64_t t_start_us = 0;
    uint64_t t_end_us = 0;
    std::vector<LidarPoint> points;
};

struct LidarConfig {
    std::string device;
    int baud = 460800;
    uint8_t sensorId = 0;
    uint64_t latencyUs = 0;       // calibrate on the actual UART; zero = uncalibrated
    uint64_t timeoutUs = 500000; // no data or sync for 0.5 s -> recovery
    std::size_t minPoints = 50;   // tune after measuring your sensor's valid return count
    uint64_t startupGraceUs = 5000000; // allow motor spin-up before timeout checks
    uint64_t retryBackoffUs = 500000;  // delay between reconnection attempts
    int readThreadPriority = 70;       // Linux SCHED_FIFO priority, 0 disables elevation
    uint64_t emptyReadSleepUs = 250;   // faster polling reduces batch-read timing variance
};

struct LidarHealth {
    uint64_t sweeps = 0;        // accepted, published sweeps
    uint64_t droppedSweeps = 0;
    uint64_t readErrors = 0;    // SDK failures + UART error indications (may overlap)
    uint64_t framingErrors = 0; // Linux TIOCGICOUNT; only if uartCountersAvailable is true
    uint64_t parityErrors = 0;
    uint64_t overrunErrors = 0;
    uint64_t checksumErrors = 0;
    uint64_t bufferOverrunErrors = 0;
    bool uartCountersAvailable = false; // not supported by all USB-UART drivers
    uint64_t syncLoss = 0;
    uint64_t callbackErrors = 0; // callback exceptions, not sensor errors
    uint64_t restarts = 0;
    double lastSweepHz = 0.0;
    uint64_t lastSweepPeriodUs = 0;
    uint64_t lastSweepJitterUs = 0; // absolute period difference vs previous sweep
    uint64_t configuredLatencyUs = 0;
};

class LidarDriver {
public:
    using ScanCallback = std::function<void(const Scan&)>;

    explicit LidarDriver(LidarConfig cfg);
    ~LidarDriver();
    LidarDriver(const LidarDriver&) = delete;
    LidarDriver& operator=(const LidarDriver&) = delete;

    // True means thread launch succeeded, NOT that hardware is connected.
    bool start(ScanCallback onScan);
    void stop();
    LidarHealth health() const;
    bool connected() const { return m_connected.load(); }

private:
    struct RawSample {
        double angleCwRad = 0.0; // SDK's raw clockwise angle
        double rangeIn = 0.0;
        uint8_t quality = 0;
    };

    void run();
    bool connectAndStart();
    void recover(bool wasStreaming);
    void shutdownConnection(bool reset);
    void onSamples(const sl_lidar_response_measurement_node_hq_t* nodes,
                   size_t count, uint64_t rxUs);
    void finishSweep(uint64_t tEndUs);
    uint64_t fitSyncTime(uint64_t index, uint64_t rawUs);
    void resetSweepState();
    void openUartCounters();
    void pollUartCounters();
    void closeUartCounters();
    void pollChecksumErrors();

    struct SyncObservation {
        uint64_t index = 0; // sample count since connect
        uint64_t rawUs = 0; // receive time worked back from its read batch
    };

    LidarConfig m_cfg;
    ScanCallback m_onScan;
    sl::IChannel* m_channel = nullptr;
    sl::ILidarDriver* m_drv = nullptr;
    float m_usPerSample = 0.0f;
    uint64_t m_tStartUs = 0;
    uint64_t m_sampleIndex = 0;
    std::deque<SyncObservation> m_syncObs; // recent sync samples, fitted to remove read jitter
    int m_lateSyncs = 0;                   // consecutive syncs far later than the fit
    uint64_t m_lastChecksumErrors = 0;     // SDK checksum count already added to health
    uint64_t m_lastSyncRxUs = 0;
    uint64_t m_lastDataRxUs = 0;
    uint64_t m_lastPeriodUs = 0;
    double m_startAngleCwRad = 0.0;
    std::vector<RawSample> m_building;
    bool m_buildingOverflow = false;

    // Separate diagnostic fd; never reads or consumes the SDK's stream data.
    int m_uartCountersFd = -1;
    uint32_t m_lastFrame = 0;
    uint32_t m_lastParity = 0;
    uint32_t m_lastOverrun = 0;
    uint32_t m_lastBufferOverrun = 0;
    uint64_t m_lastUartPollUs = 0;
    bool m_reportedUartCountersUnavailable = false;

    std::thread m_thread;
    std::atomic<bool> m_running{false};
    std::atomic<bool> m_connected{false};
    mutable std::mutex m_healthMutex;
    LidarHealth m_health;
};
