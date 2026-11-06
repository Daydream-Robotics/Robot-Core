// lidar.cpp
#include "lidar.hpp"
#include "pros_serial_io.hpp"
#include "pros/rtos.hpp"
#include <atomic>

namespace {
    RPLidarLite* g_lidar = nullptr;
    pros::Mutex* g_mutex = nullptr;
    std::atomic<bool> g_stopRequested(false);

    void lidarTaskFn(void*) {
        // Keep trying until the lidar answers (e.g. it's still powering up).
        bool started = false;
        while (!started && !g_stopRequested) {
            g_mutex->take(TIMEOUT_MAX);
            if (!g_stopRequested) started = g_lidar->startScan();
            g_mutex->give();
            if (!started) pros::delay(1000);
        }

        while (!g_stopRequested) {
            g_mutex->take(TIMEOUT_MAX);
            if (!g_stopRequested) g_lidar->update();
            g_mutex->give();
            pros::delay(5);
        }
    }
}  // namespace

void lidarInit(uint8_t smartPort, int32_t baud) {
    if (g_lidar) return;
    // Static locals: created here at runtime, and stored outside any task stack.
    static ProsSerialIO serial(smartPort, baud);
    static RPLidarLite lidar(serial);
    static pros::Mutex mutex;
    g_lidar = &lidar;
    g_mutex = &mutex;
    pros::Task task(lidarTaskFn, nullptr, "lidar");
}

int lidarGetScan(LidarPoint* out, int maxPoints) {
    if (!g_lidar) return 0;
    g_mutex->take(TIMEOUT_MAX);
    int n = g_lidar->getScan(out, maxPoints);
    g_mutex->give();
    return n;
}

uint32_t lidarScanCount() {
    if (!g_lidar) return 0;
    g_mutex->take(TIMEOUT_MAX);
    uint32_t c = g_lidar->scanCount();
    g_mutex->give();
    return c;
}

void lidarStop() {
    if (!g_lidar) return;
    g_stopRequested = true;
    g_mutex->take(TIMEOUT_MAX);
    g_lidar->stop();
    g_mutex->give();
}


