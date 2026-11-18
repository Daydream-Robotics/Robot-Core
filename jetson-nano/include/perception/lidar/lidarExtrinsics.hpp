#pragma once

#include <cstdint>
#include <vector>

#include "perception/lidar/lidarDriver.hpp"

/**
 * @brief Blocked bearing range in the sensor frame (rad, CCW+). May wrap across ±pi.
 */
struct LidarBearingMask {
    double start_rad = 0.0; ///< start of the blocked range (rad)
    double end_rad = 0.0; ///< end of the blocked range (rad)
};

/**
 * @brief Axis-aligned robot footprint around the EKF origin (in).
 *
 * Covers x in [-rear_in, front_in] and y in [-right_in, left_in], grown by margin_in.
 */
struct LidarRobotFootprint {
    double front_in = 0.0; ///< origin to the front edge (in)
    double rear_in = 0.0; ///< origin to the rear edge (in)
    double left_in = 0.0; ///< origin to the left edge (in)
    double right_in = 0.0; ///< origin to the right edge (in)
    double margin_in = 0.0; ///< extra clearance on every side (in)
};

/**
 * @brief One sensor's mount and self-filter settings.
 */
struct LidarMount {
    uint8_t sensor_id = 0; ///< matches Scan::sensor_id
    double x_in = 0.0; ///< sensor position, robot frame (+X forward) (in)
    double y_in = 0.0; ///< sensor position, robot frame (+Y left) (in)
    double psi_rad = 0.0; ///< sensor yaw in the robot frame, CCW+, includes the C1's 0 deg offset (rad)

    double r_min_in = 0.0; ///< closer points are dropped (in)
    double r_max_in = 0.0; ///< farther points are dropped (in)
    double mount_height_in = 0.0; ///< scan plane height for #144, unused by the 2D transform (in)

    std::vector<LidarBearingMask> blocked_bearings; ///< bearings occluded by the robot
    LidarRobotFootprint footprint; ///< points inside are dropped
};

/**
 * @brief One point in the robot frame.
 */
struct RobotPoint {
    double x_in = 0.0; ///< robot frame, +X forward (in)
    double y_in = 0.0; ///< robot frame, +Y left (in)
    uint64_t stamp_us = 0; ///< the point's own monotonic timestamp, kept for deskew (us)
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

/**
 * @brief Every sensor's mount and self-filter settings, from the [LIDAR] section of constants.h.
 *
 * @return std::vector<LidarMount> One mount per sensor, in sensor_id order.
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
     * @return RobotScan Kept points in the robot frame. Throws std::invalid_argument if scan.sensor_id doesn't match.
     */
    RobotScan toRobotFrame(const Scan& scan) const;

private:
    /**
     * @brief Checks a sensor-frame bearing against the blocked ranges.
     *
     * @param bearing_rad Bearing in the sensor frame (rad, CCW+).
     * @return bool True if blocked.
     */
    bool masked(double bearing_rad) const;

    /**
     * @brief Checks a robot-frame point against the footprint plus margin.
     *
     * @param x_in Robot frame x (in).
     * @param y_in Robot frame y (in).
     * @return bool True if inside.
     */
    bool insideFootprint(double x_in, double y_in) const;

    LidarMount m_mount; ///< this sensor's mount and filter settings
};
