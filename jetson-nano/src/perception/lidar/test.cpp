// test.cpp - test RPLidarLite on a PC or the Jetson Nano over USB.
// Usage: rplidar_lite_test [serial port] [baud] [points]
//   Linux:   rplidar_lite_test /dev/ttyUSB0 460800
//   Windows: rplidar_lite_test COM15 460800
//   Add "points" as a third argument to also print every point with its time.
//
// Ctrl+C, or closing the terminal, stops the motor properly.
//
// The lidar is read in its own thread (like the PROS version), so slow printing
// never delays reading. If printing can't keep up, revolutions are skipped for
// display only; the data and timestamps stay correct.
#include "rplidar_lite.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <thread>

#ifdef _WIN32
  #include <windows.h>
  #include "win_serial_io.hpp"
  using PlatformSerialIO = WinSerialIO;
  static const char* kDefaultPort = "COM15";
#else
  #include <csignal>
  #include "linux_serial_io.hpp"
  using PlatformSerialIO = LinuxSerialIO;
  static const char* kDefaultPort = "/dev/ttyUSB0";
#endif

static std::atomic<bool> g_stopRequested(false);

// ---- Stop handling: make sure the motor is stopped however the program ends ----
#ifdef _WIN32
static std::atomic<bool> g_cleanedUp(false);

static BOOL WINAPI onConsoleEvent(DWORD event) {
    g_stopRequested = true;
    if (event == CTRL_CLOSE_EVENT) {
        // Windows ends the process as soon as we return, so wait for main() to stop the lidar.
        for (int i = 0; i < 300 && !g_cleanedUp; ++i) Sleep(10);
    }
    return TRUE;
}
static void installStopHandler() { SetConsoleCtrlHandler(onConsoleEvent, TRUE); }
static void markCleanedUp() { g_cleanedUp = true; }
#else
// Only sets a flag: that is all a signal handler can safely do.
// main() then leaves its loop and stops the lidar normally.
static void onSignal(int) { g_stopRequested = true; }

static void installStopHandler() {
    std::signal(SIGINT, onSignal);   // Ctrl+C
    std::signal(SIGTERM, onSignal);  // kill, or a service manager stopping the program
    std::signal(SIGHUP, onSignal);   // terminal window closed
}
static void markCleanedUp() {}
#endif

static void sleepMs(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main(int argc, char* argv[]) {
    const char* port = argc > 1 ? argv[1] : kDefaultPort;
    uint32_t baud = argc > 2 ? (uint32_t)strtoul(argv[2], nullptr, 10) : 460800;
    bool printPoints = argc > 3 && strcmp(argv[3], "points") == 0;

    PlatformSerialIO serial;
    if (!serial.open(port, baud)) {
        printf("Could not open %s. Is the port right, and is no other program using it?\n", port);
#ifndef _WIN32
        printf("On Linux you also need permission: sudo usermod -aG dialout $USER, then log in again.\n");
#endif
        return 1;
    }

    RPLidarLite lidar(serial);

    LidarDeviceInfo info;
    if (!lidar.getDeviceInfo(info)) {
        printf("No reply from the lidar on %s at %u baud. Check the baud rate.\n", port, baud);
        serial.close();
        return 1;
    }
    printf("Connected. Model 0x%02X, firmware %d.%02d, hardware %d\nSerial: ",
           info.model, info.firmwareMajor, info.firmwareMinor, info.hardware);
    for (int i = 0; i < 16; ++i) printf("%02X", info.serial[i]);
    printf("\n");

    installStopHandler();

    if (!lidar.startScan()) {
        printf("Lidar did not start scanning.\n");
        serial.close();
        return 1;
    }
    printf("Scanning. Press Ctrl+C to stop.\n\n");

    static LidarPoint scan[RPLidarLite::kMaxPoints];
    const uint64_t programStartUs = serial.micros();   // times are printed relative to this

    // Buffer console output so printing hundreds of lines per revolution is faster.
    static char outBuf[1 << 16];
    setvbuf(stdout, outBuf, _IOFBF, sizeof(outBuf));

    // The reader thread and main() share the lidar, so every access goes through this lock.
    std::mutex lidarLock;
    std::thread reader([&] {
        while (!g_stopRequested) {
            {
                std::lock_guard<std::mutex> guard(lidarLock);
                lidar.update();
            }
            sleepMs(1);
        }
    });

    uint32_t lastPrinted = 0;
    while (!g_stopRequested) {
        // Grab the newest revolution (if there is one) while holding the lock briefly.
        LidarScanTiming timing;
        int n = 0;
        uint32_t rev;
        uint32_t badPackets;
        {
            std::lock_guard<std::mutex> guard(lidarLock);
            rev = lidar.scanCount();
            badPackets = lidar.badPacketCount();
            if (rev != lastPrinted) n = lidar.getScan(scan, RPLidarLite::kMaxPoints, &timing);
        }

        if (rev == lastPrinted) { sleepMs(2); continue; }
        uint32_t skipped = lastPrinted ? rev - lastPrinted - 1 : 0;
        lastPrinted = rev;

        // Find the closest point and the point nearest to straight ahead (0 deg).
        int closest = -1, front = -1;
        float bestFrontDiff = 999.0f;
        for (int i = 0; i < n; ++i) {
            if (closest < 0 || scan[i].distanceMm < scan[closest].distanceMm) closest = i;
            float diff = scan[i].angleDeg > 180.0f ? 360.0f - scan[i].angleDeg : scan[i].angleDeg;
            if (diff < bestFrontDiff) { bestFrontDiff = diff; front = i; }
        }

        double startSec = (double)(timing.startUs - programStartUs) / 1e6;
        double durationMs = (double)(timing.endUs - timing.startUs) / 1e3;
        double hz = durationMs > 0 ? 1000.0 / durationMs : 0.0;

        printf("[%9.3f s] Rev %5u | %4d pts | %5.1f ms (%4.1f Hz)",
               startSec, rev, n, durationMs, hz);
        if (front >= 0)
            printf(" | front: %6.0f mm", scan[front].distanceMm);
        if (closest >= 0)
            printf(" | closest: %6.0f mm at %5.1f deg", scan[closest].distanceMm, scan[closest].angleDeg);
        printf(" | bad pkts: %u", badPackets);
        if (skipped) printf(" | (%u revs not shown)", skipped);
        printf("\n");

        if (printPoints) {
            for (int i = 0; i < n; ++i) {
                printf("    t=%10.4f s  angle=%6.2f deg  dist=%7.1f mm  q=%2d\n",
                       (double)(scan[i].timeUs - programStartUs) / 1e6,
                       scan[i].angleDeg, scan[i].distanceMm, scan[i].quality);
            }
        }
        fflush(stdout);
    }

    reader.join();

    printf("\nStopping lidar...\n");
    lidar.stop();
    serial.close();
    printf("Stopped.\n");
    fflush(stdout);
    markCleanedUp();
    return 0;
}