#pragma once

#include <cstdint>
#include <vector>

/**
 * @brief One point in the robot frame.
 */
struct RobotPoint {
    double xIn = 0.0; ///< robot frame, sideways (+X at heading 0) (in)
    double yIn = 0.0; ///< robot frame, forward (+Y at heading 0) (in)
    uint64_t stampUs = 0; ///< the point's own monotonic timestamp
};

/**
 * @brief One sensor's scan after self-filtering, in the robot frame.
 */
struct RobotScan {
    uint8_t sensorId = 0; ///< sensor that took the scan
    uint64_t tStartUs = 0; ///< first point's time (us)
    uint64_t tEndUs = 0; ///< last point's time (us)
    std::vector<RobotPoint> points; ///< kept points
};
