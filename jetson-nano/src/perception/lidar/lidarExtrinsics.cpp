#include "perception/lidar/lidarExtrinsics.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <utility>

#include "config/constants.h"
#include "utils/mathHelpers.hpp"

namespace {

constexpr double ANGLE_EPSILON = 1e-12; // tolerance on mask edges (rad)

void validate(const LidarMount& mount) {
    const auto& f = mount.footprint;
    if (!std::isfinite(mount.x_in) || !std::isfinite(mount.y_in) ||
        !std::isfinite(mount.psi_rad) || !std::isfinite(mount.mount_height_in) ||
        mount.mount_height_in < 0.0 || !std::isfinite(mount.r_min_in) ||
        !std::isfinite(mount.r_max_in) || mount.r_min_in < 0.0 ||
        mount.r_max_in <= mount.r_min_in) {
        throw std::invalid_argument("invalid lidar pose, height or distance limits");
    }
    for (const double v : {f.front_in, f.rear_in, f.left_in, f.right_in, f.margin_in}) {
        if (!std::isfinite(v) || v < 0.0) {
            throw std::invalid_argument("footprint dimensions and margin must be finite and >= 0");
        }
    }
    for (const auto& mask : mount.blocked_bearings) {
        if (!std::isfinite(mask.start_rad) || !std::isfinite(mask.end_rad) ||
            mask.start_rad < -M_PI || mask.start_rad > M_PI ||
            mask.end_rad < -M_PI || mask.end_rad > M_PI ||
            mask.start_rad == mask.end_rad) {
            throw std::invalid_argument("invalid sensor bearing mask");
        }
    }
}

} // namespace

std::vector<LidarMount> lidarMounts() {
    const LidarRobotFootprint footprint{LIDAR_FOOTPRINT_FRONT_IN, LIDAR_FOOTPRINT_REAR_IN,
                                        LIDAR_FOOTPRINT_LEFT_IN, LIDAR_FOOTPRINT_RIGHT_IN,
                                        LIDAR_FOOTPRINT_MARGIN_IN};

    // TODO: add blocked bearings once the chassis and brackets exist
    std::vector<LidarMount> mounts = {
        {0, LIDAR_0_X_IN, LIDAR_0_Y_IN, LIDAR_0_PSI_RAD, LIDAR_R_MIN_IN, LIDAR_R_MAX, LIDAR_0_HEIGHT_IN, {}, footprint},
        {1, LIDAR_1_X_IN, LIDAR_1_Y_IN, LIDAR_1_PSI_RAD, LIDAR_R_MIN_IN, LIDAR_R_MAX, LIDAR_1_HEIGHT_IN, {}, footprint},
    };
    for (const LidarMount& mount : mounts) {
        validate(mount);
    }
    return mounts;
}

LidarTransform::LidarTransform(LidarMount mount) : m_mount(std::move(mount)) {
    validate(m_mount);
}

RobotScan LidarTransform::toRobotFrame(const Scan& scan) const {
    if (scan.sensor_id != m_mount.sensor_id) {
        std::fprintf(stderr, "[LiDAR %u] scan sent to the transform for sensor %u, dropped\n",
                     unsigned(scan.sensor_id), unsigned(m_mount.sensor_id));
        return RobotScan{};
    }

    RobotScan result;
    result.sensor_id = scan.sensor_id;
    result.t_start_us = scan.t_start_us;
    result.t_end_us = scan.t_end_us;
    result.points.reserve(scan.points.size());

    const double c = std::cos(m_mount.psi_rad);
    const double s = std::sin(m_mount.psi_rad);

    for (const auto& point : scan.points) {
        // bearing_rad is already CCW-positive
        const double r = point.range_in;
        const double beta = point.bearing_rad;
        if (!std::isfinite(r) || !std::isfinite(beta) ||
            r < m_mount.r_min_in || r > m_mount.r_max_in) {

            continue;
        }
        // masks are in the sensor frame
        if (masked(beta)) continue;

        const double sensorX = r * std::cos(beta);
        const double sensorY = r * std::sin(beta);
        const double x = m_mount.x_in + c * sensorX - s * sensorY;
        const double y = m_mount.y_in + s * sensorX + c * sensorY;
        if (insideFootprint(x, y)) continue;

        result.points.push_back({x, y, point.stamp_us});
    }
    return result;
}

bool LidarTransform::masked(double bearing_rad) const {
    const double angle = utils::normalizeAngle(bearing_rad);
    for (const auto& mask : m_mount.blocked_bearings) {
        // a -180 to 180 mask blocks the whole circle
        if ((mask.start_rad == -M_PI && mask.end_rad == M_PI) ||
            (mask.start_rad == M_PI && mask.end_rad == -M_PI)) return true;
        const double begin = utils::normalizeAngle(mask.start_rad);
        const double end = utils::normalizeAngle(mask.end_rad);
        if (begin <= end) {
            if (angle >= begin - ANGLE_EPSILON && angle <= end + ANGLE_EPSILON) return true;
        } else { // wraps across +/-pi
            if (angle >= begin - ANGLE_EPSILON || angle <= end + ANGLE_EPSILON) return true;
        }
    }
    return false;
}

bool LidarTransform::insideFootprint(double x_in, double y_in) const {
    const auto& f = m_mount.footprint;
    return x_in >= -f.rear_in - f.margin_in &&
           x_in <=  f.front_in + f.margin_in &&
           y_in >= -f.right_in - f.margin_in &&
           y_in <=  f.left_in + f.margin_in;
}
