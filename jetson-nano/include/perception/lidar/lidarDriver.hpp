#pragma once

// SDK-based driver for one physical Slamtec sensor. Windows PC or Linux/Jetson Nano.
// This header deliberately follows the interface from Robot-Core issue #137.
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
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
};

struct LidarHealth {
    uint64_t sweeps = 0;        // accepted, published sweeps
    uint64_t droppedSweeps = 0;
    uint64_t readErrors = 0;    // SDK API errors, NOT raw UART framing error count
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
    void recover();
    void shutdownConnection(bool reset);
    void onSamples(const sl_lidar_response_measurement_node_hq_t* nodes,
                   size_t count, uint64_t rxUs);
    void finishSweep(uint64_t tEndUs);
    void resetSweepState();

    LidarConfig m_cfg;
    ScanCallback m_onScan;
    sl::IChannel* m_channel = nullptr;
    sl::ILidarDriver* m_drv = nullptr;
    float m_usPerSample = 0.0f;
    uint64_t m_tStartUs = 0;
    uint64_t m_lastNodeUs = 0;
    uint64_t m_lastSyncRxUs = 0;
    uint64_t m_lastDataRxUs = 0;
    uint64_t m_lastPeriodUs = 0;
    double m_startAngleCwRad = 0.0;
    std::vector<RawSample> m_building;
    bool m_buildingOverflow = false;

    std::thread m_thread;
    std::atomic<bool> m_running{false};
    std::atomic<bool> m_connected{false};
    mutable std::mutex m_healthMutex;
    LidarHealth m_health;
};
