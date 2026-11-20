#include "perception/lidar/lidarExtrinsics.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <utility>

#include "config/constants.h"
#include "utils/mathHelpers.hpp"

namespace {

void validate(const LidarMount& mount) {
    const auto& f = mount.footprint;
    if (!std::isfinite(mount.xIn) || !std::isfinite(mount.yIn) ||
        !std::isfinite(mount.psiRad) || !std::isfinite(mount.mountHeightIn) ||
        mount.mountHeightIn < 0.0 || !std::isfinite(mount.rMinIn) ||
        !std::isfinite(mount.rMaxIn) || mount.rMinIn < 0.0 ||
        mount.rMaxIn <= mount.rMinIn) {
        throw std::invalid_argument("invalid lidar pose, height or distance limits");
    }
    for (const double v : {f.frontIn, f.rearIn, f.leftIn, f.rightIn, f.marginIn}) {
        if (!std::isfinite(v) || v < 0.0) {
            throw std::invalid_argument("footprint dimensions and margin must be finite and >= 0");
        }
    }
    for (const auto& mask : mount.blockedBearings) {
        if (!std::isfinite(mask.startRad) || !std::isfinite(mask.endRad) ||
            mask.startRad < -M_PI || mask.startRad > M_PI ||
            mask.endRad < -M_PI || mask.endRad > M_PI ||
            mask.startRad == mask.endRad) {
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
    if (scan.sensorId != m_mount.sensorId) {
        std::fprintf(stderr, "[LiDAR %u] scan sent to the transform for sensor %u, dropped\n",
                     unsigned(scan.sensorId), unsigned(m_mount.sensorId));
        return RobotScan{};
    }

    RobotScan result;
    result.sensorId = scan.sensorId;
    result.tStartUs = scan.tStartUs;
    result.tEndUs = scan.tEndUs;
    result.points.reserve(scan.points.size());

    const double c = std::cos(m_mount.psiRad);
    const double s = std::sin(m_mount.psiRad);

    for (const auto& point : scan.points) {
        // bearingRad is already CCW-positive
        const double r = point.rangeIn;
        const double beta = point.bearingRad;
        if (!std::isfinite(r) || !std::isfinite(beta) ||
            r < m_mount.rMinIn || r > m_mount.rMaxIn) {

            continue;
        }
        // masks are in the sensor frame
        if (masked(beta)) continue;

        const double sensorX = r * std::cos(beta);
        const double sensorY = r * std::sin(beta);
        const double x = m_mount.xIn + c * sensorX - s * sensorY;
        const double y = m_mount.yIn + s * sensorX + c * sensorY;
        if (insideFootprint(x, y)) continue;

        result.points.push_back({x, y, point.stampUs});
    }
    return result;
}

bool LidarTransform::masked(double bearingRad) const {
    const double angle = utils::normalizeAngle(bearingRad);
    for (const auto& mask : m_mount.blockedBearings) {
        // a -180 to 180 mask blocks the whole circle
        if ((mask.startRad == -M_PI && mask.endRad == M_PI) ||
            (mask.startRad == M_PI && mask.endRad == -M_PI)) return true;
        const double begin = utils::normalizeAngle(mask.startRad);
        const double end = utils::normalizeAngle(mask.endRad);
        if (begin <= end) {
            if (angle >= begin - LIDAR_MASK_EPSILON_RAD && angle <= end + LIDAR_MASK_EPSILON_RAD) return true;
        } else { // wraps across +/-pi
            if (angle >= begin - LIDAR_MASK_EPSILON_RAD || angle <= end + LIDAR_MASK_EPSILON_RAD) return true;
        }
    }
    return false;
}

bool LidarTransform::insideFootprint(double xIn, double yIn) const {
    const auto& f = m_mount.footprint;
    return xIn >= -f.rearIn - f.marginIn &&
           xIn <=  f.frontIn + f.marginIn &&
           yIn >= -f.rightIn - f.marginIn &&
           yIn <=  f.leftIn + f.marginIn;
}
