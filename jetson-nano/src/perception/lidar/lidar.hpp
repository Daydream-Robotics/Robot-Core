// lidar.hpp - runs the lidar in its own PROS task
#pragma once
#include "rplidar_lite.hpp"

// Call once from initialize(). Starts a background task that keeps the lidar running.
void lidarInit(uint8_t smartPort, int32_t baud = 460800);

// Copy the latest full revolution into `out`. Returns the number of points.
// Make `out` a global or static array: 2048 points is too big for a task's stack.
int lidarGetScan(LidarPoint* out, int maxPoints);

// Number of revolutions received so far (use it to tell when a new scan is available).
uint32_t lidarScanCount();

// Stop scanning and the motor.
void lidarStop();
