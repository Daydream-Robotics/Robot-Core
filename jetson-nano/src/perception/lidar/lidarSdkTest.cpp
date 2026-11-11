// Minimal hardware test: prints accepted sweeps, point timestamp validation,
// sweep period/jitter and driver counters. One sensor or two independent sensors.
// Usage: lidar_sdk_test <portA> [portB|-] [seconds] [latencyA_us] [latencyB_us]
// Example: lidar_sdk_test.exe COM15 - 300 0 (Windows PC)
// Example: lidar_sdk_test /dev/ttyUSB0 - 300 0 (Linux PC / Jetson)
// Example: lidar_sdk_test /dev/ttyTHS1 /dev/ttyTHS2 300 120 130
// Set the latency numbers ONLY after measuring on the actual UART.
#include "perception/lidar/lidarDriver.hpp"
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <thread>

namespace {
volatile std::sig_atomic_t stopSignal = 0;
void onSignal(int) { stopSignal = 1; }
struct Counters {
    std::atomic<uint64_t> scans{0};
    std::atomic<uint64_t> invalidTime{0};
    std::atomic<uint64_t> lastPoints{0};
};
void acceptScan(const Scan& scan, Counters& counter) {
    uint64_t previous = scan.t_start_us;
    if (scan.t_end_us <= scan.t_start_us) ++counter.invalidTime;
    for (const auto& point : scan.points) {
        if (point.stamp_us < previous || point.stamp_us >= scan.t_end_us)
            ++counter.invalidTime;
        previous = point.stamp_us;
    }
    counter.lastPoints = scan.points.size();
    ++counter.scans;
}
void report(const LidarDriver& drv, const Counters& c, const char* name) {
    auto h = drv.health();
    std::printf("%s connected=%d sweeps=%llu dropped=%llu samples=%llu "
        "rate=%.2f Hz period=%llu us jitter=%llu us latency_cfg=%llu us "
        "sdk_errors=%llu callback_errors=%llu sync_loss=%llu restarts=%llu bad_stamps=%llu\n",
        name, drv.connected(),
        static_cast<unsigned long long>(h.sweeps),
        static_cast<unsigned long long>(h.droppedSweeps),
        static_cast<unsigned long long>(c.lastPoints.load()),
        h.lastSweepHz,
        static_cast<unsigned long long>(h.lastSweepPeriodUs),
        static_cast<unsigned long long>(h.lastSweepJitterUs),
        static_cast<unsigned long long>(h.configuredLatencyUs),
        static_cast<unsigned long long>(h.readErrors),
        static_cast<unsigned long long>(h.callbackErrors),
        static_cast<unsigned long long>(h.syncLoss),
        static_cast<unsigned long long>(h.restarts),
        static_cast<unsigned long long>(c.invalidTime.load()));
}
} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "Usage: %s <portA> [portB|-] [seconds] [latencyA_us] [latencyB_us]\n", argv[0]);
        return 2;
    }
    const char* portB = argc > 2 ? argv[2] : "-";
    const int seconds = argc > 3 ? std::atoi(argv[3]) : 300;
    const uint64_t latencyA = argc > 4 ? std::strtoull(argv[4], nullptr, 10) : 0;
    const uint64_t latencyB = argc > 5 ? std::strtoull(argv[5], nullptr, 10) : 0;
    if (seconds < 1) return 2;
    Counters a{}, b{};
    LidarDriver sensorA({argv[1], 460800, 0, latencyA, 2000000, 50});
    LidarDriver sensorB({portB, 460800, 1, latencyB, 2000000, 50});
    if (!sensorA.start([&](const Scan& scan) { acceptScan(scan, a); })) return 1;
    const bool second = std::string(portB) != "-";
    if (second && !sensorB.start([&](const Scan& scan) { acceptScan(scan, b); })) {
        sensorA.stop(); return 1;
    }
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    std::puts("Testing one/two sensors. 300-second two-sensor motor-noise and TX recovery tests require robot hardware.");
    for (int i = 0; i < seconds && !stopSignal; ++i) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        report(sensorA, a, "A");
        if (second) report(sensorB, b, "B");
        std::fflush(stdout);
    }
    sensorA.stop();
    if (second) sensorB.stop();
    report(sensorA, a, "FINAL A");
    if (second) report(sensorB, b, "FINAL B");
    return a.invalidTime != 0 || (second && b.invalidTime != 0) ? 1 : 0;
}
