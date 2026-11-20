#pragma once

#include <cstdint>
#include <vector>

/**
 * @brief One point in the robot frame.
 */
struct RobotPoint {
    double x_in = 0.0; ///< robot frame, +X forward (in)
    double y_in = 0.0; ///< robot frame, +Y left (in)
    uint64_t stamp_us = 0; ///< the point's own monotonic timestamp
};

/**
 * @brief One sensor's scan after self-filtering, in the robot frame.
 */
struct RobotScan {
    uint8_t sensor_id = 0; ///< sensor that took the scan
    uint64_t t_start_us = 0; ///< first point's time (us)
    uint64_t t_end_us = 0; ///< last point's time (us)
    std::vector<RobotPoint> points; ///< kept points
};
