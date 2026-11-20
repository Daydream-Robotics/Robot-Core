#pragma once

#include <cstdint>
#include <vector>

#include "perception/lidar/lidarDriver.hpp"
#include "perception/lidar/lidarTypes.hpp"

/**
 * @brief Blocked bearing range in the sensor frame (rad, CCW+). May wrap across ±pi.
 */
struct LidarBearingMask {
    double startRad = 0.0; ///< start of the blocked range (rad)
    double endRad = 0.0; ///< end of the blocked range (rad)
};

/**
 * @brief Axis-aligned robot footprint around the EKF origin (in).
 *
 * Covers x in [-rearIn, frontIn] and y in [-rightIn, leftIn], grown by marginIn.
 */
struct LidarRobotFootprint {
    double frontIn = 0.0; ///< origin to the front edge (in)
    double rearIn = 0.0; ///< origin to the rear edge (in)
    double leftIn = 0.0; ///< origin to the left edge (in)
    double rightIn = 0.0; ///< origin to the right edge (in)
    double marginIn = 0.0; ///< extra clearance on every side (in)
};

/**
 * @brief One sensor's mount and self-filter settings.
 */
struct LidarMount {
    uint8_t sensorId = 0; ///< matches Scan::sensorId
    double xIn = 0.0; ///< sensor position, robot frame (+X forward) (in)
    double yIn = 0.0; ///< sensor position, robot frame (+Y left) (in)
    double psiRad = 0.0; ///< sensor yaw in the robot frame, CCW+, includes the C1's 0 deg offset (rad)

    double rMinIn = 0.0; ///< closer points are dropped (in)
    double rMaxIn = 0.0; ///< farther points are dropped (in)
    double mountHeightIn = 0.0; ///< scan plane height for #144, unused by the 2D transform (in)

    std::vector<LidarBearingMask> blockedBearings; ///< bearings occluded by the robot
    LidarRobotFootprint footprint; ///< points inside are dropped
};

/**
 * @brief Every sensor's mount and self-filter settings, from the [LIDAR] section of constants.h.
 *
 * @return std::vector<LidarMount> One mount per sensor, in sensorId order.
 */
std::vector<LidarMount> lidarMounts();

/**
 * @brief Converts one sensor's scans to the robot frame and drops points that hit the robot.
 */
class LidarTransform {
public:
    /**
     * @brief Constructs the transform for one sensor.
     *
     * @param mount The sensor's mount and filter settings. Throws std::invalid_argument if invalid.
     */
    explicit LidarTransform(LidarMount mount);

    /**
     * @brief Self-filters the scan and converts it to the robot frame, keeping timestamps for deskew (#142).
     *
     * @param scan Scan from this transform's sensor.
     * @return RobotScan Kept points in the robot frame. Empty, with an error logged, if scan.sensorId doesn't match.
     */
    RobotScan toRobotFrame(const Scan& scan) const;

private:
    /**
     * @brief Checks a sensor-frame bearing against the blocked ranges.
     *
     * @param bearingRad Bearing in the sensor frame (rad, CCW+).
     * @return bool True if blocked.
     */
    bool masked(double bearingRad) const;

    /**
     * @brief Checks a robot-frame point against the footprint plus margin.
     *
     * @param xIn Robot frame x (in).
     * @param yIn Robot frame y (in).
     * @return bool True if inside.
     */
    bool insideFootprint(double xIn, double yIn) const;

    LidarMount m_mount; ///< this sensor's mount and filter settings
};
