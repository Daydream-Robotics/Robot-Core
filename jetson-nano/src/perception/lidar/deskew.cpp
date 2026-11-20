#include "perception/lidar/deskew.hpp"

#include <algorithm>
#include <cmath>

#include "config/constants.h"
#include "utils/mathHelpers.hpp"

uint64_t ScanMerger::deskewScan(const RobotScan& scan, const PoseBuffer& poses, const Pose& ref,
                                std::vector<DeskewedPoint>& out) {
    // points with no pose in the buffer
    uint64_t dropped = 0;

    // rotation into the reference frame
    const double cRef = std::cos(-ref.theta);
    const double sRef = std::sin(-ref.theta);

    for (const RobotPoint& p : scan.points) {
        // robot pose when this point was measured
        const std::optional<Pose> pose = poses.poseAt(p.stampUs);

        // no pose, drop the point
        if (pose == std::nullopt) {
            ++dropped;
            continue;
        }

        // heading change from the point's time to the reference
        const double dTheta = utils::normalizeAngle(pose->theta - ref.theta);
        const double c = std::cos(dTheta);
        const double s = std::sin(dTheta);

        // rotate the point by the heading change
        const double rx = c * p.xIn - s * p.yIn;
        const double ry = s * p.xIn + c * p.yIn;

        // robot motion from the point's time to the reference, world frame
        const double dx = pose->x - ref.x;
        const double dy = pose->y - ref.y;

        // that motion in the reference robot frame
        const double tx = cRef * dx - sRef * dy;
        const double ty = sRef * dx + cRef * dy;

        // deskewed point, tagged with its sensor
        out.push_back(DeskewedPoint{rx + tx, ry + ty, scan.sensorId});
    }
    return dropped;
}

void ScanMerger::addScan(const RobotScan& scan) {
    // drop an empty scan or unknown sensor
    if (scan.points.empty() || scan.sensorId >= LIDAR_NUM_SENSORS) {
        return;
    }

    // driver threads write, build() reads
    std::lock_guard<std::mutex> lock(m_mutex);

    // drop sweep that isn't newer than the stored one
    const std::optional<RobotScan>& stored = m_latest[scan.sensorId];
    if (stored.has_value() && scan.tEndUs <= stored->tEndUs) {
        return;
    }

    // keep this sensor's newest sweep
    m_latest[scan.sensorId] = scan;
}

std::optional<DeskewedCloud> ScanMerger::build(const PoseBuffer& poses) {
    // copy the sweeps
    std::array<std::optional<RobotScan>, LIDAR_NUM_SENSORS> latest;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        latest = m_latest;
    }

    // atore newest sweep end
    uint64_t tRef = 0;
    for (const std::optional<RobotScan>& s : latest) {
        if (s.has_value()) {
            tRef = std::max(tRef, s->tEndUs);
        }
    }

    // no sweeps yet, or nothing new since the last cloud
    if (tRef == 0 || tRef <= m_lastRefUs) {
        return std::nullopt;
    }

    // odom not caught up to tRef ye
    const std::optional<Pose> ref = poses.poseAt(tRef);
    if (ref == std::nullopt) {
        return std::nullopt;
    }

    DeskewedCloud cloud;
    cloud.tRefUs = tRef;

    // a sweep older than this is stale
    const uint64_t staleUs = static_cast<uint64_t>(LIDAR_STALE_SWEEPS) * LIDAR_SWEEP_US;

    // deskew every fresh sweep to tRef
    for (const std::optional<RobotScan>& s : latest) {
        if (!s.has_value() || tRef - s->tEndUs > staleUs) {
            cloud.singleSensor = true;
            continue;
        }
        cloud.dropped += deskewScan(*s, poses, *ref, cloud.points);
    }

    // mark this frame as built
    m_lastRefUs = tRef;
    return cloud;
}
