

#pragma once

#include "perception/lidar/lidarDriver.hpp"

#include <cstdint>
#include <string>
#include <vector>

// Sensor-frame bearing interval, in radians. Bearings are already CCW-positive
// in lidarDriver.hpp. Intervals may cross the -pi / +pi boundary.
struct LidarBearingMask {
    double start_rad = 0.0;
    double end_rad = 0.0;
};

// Axis-aligned robot footprint relative to the EKF/robot origin, in inches.
// The rectangle covers x in [-rear_in, front_in], y in [-right_in, left_in].
struct LidarRobotFootprint {
    double front_in = 0.0;
    double rear_in = 0.0;
    double left_in = 0.0;
    double right_in = 0.0;
    double margin_in = 0.0;
};

// One sensor's mounting position and self-filter configuration.
struct LidarMount {
    uint8_t sensor_id = 0;
    double x_in = 0.0;
    double y_in = 0.0;
    double psi_rad = 0.0; // Robot frame: +X forward, +Y left, rotation CCW+.

    double r_min_in = 0.0;
    double r_max_in = 0.0;
    double mount_height_in = 0.0; // Recorded for issue #144; 2D transform ignores Z.

    std::vector<LidarBearingMask> blocked_bearings;
    LidarRobotFootprint footprint;
};

struct RobotPoint {
    double x_in = 0.0;
    double y_in = 0.0;
    uint64_t stamp_us = 0; // Keep each point's original monotonic timestamp.
};

struct RobotScan {
    uint8_t sensor_id = 0;
    uint64_t t_start_us = 0;
    uint64_t t_end_us = 0;
    std::vector<RobotPoint> points;
};

// Returns false and logs a useful error on invalid/missing configuration.
// On failure, 'out' is left unchanged. Each sensor gets the robot footprint.
bool loadLidarMounts(const std::string& path, std::vector<LidarMount>& out);

class LidarTransform {
public:
    explicit LidarTransform(LidarMount mount);

    // Filter and transform a Scan into the common robot frame, preserving
    // all timestamps for later motion deskewing (issue #142).
    // Throws std::invalid_argument if scan.sensor_id does not match the mount.
    RobotScan toRobotFrame(const Scan& scan) const;

private:
    bool masked(double bearing_rad) const;
    bool insideFootprint(double x_in, double y_in) const;

    LidarMount m_mount;
};