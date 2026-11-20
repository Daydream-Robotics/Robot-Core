#include "perception/lidar/deskew.hpp"

#include <algorithm>
#include <cmath>

#include "config/constants.h"
#include "utils/mathHelpers.hpp"

uint64_t deskewScan(const RobotScan& scan, const PoseBuffer& poses, const Pose& ref, std::vector<DeskewedPoint>& out) {
    // points with no pose in the buffer
    uint64_t dropped = 0;

    // rotation into the reference frame
    const double cRef = std::cos(-ref.theta);
    const double sRef = std::sin(-ref.theta);

    for (const RobotPoint& p : scan.points) {
        // robot pose when this point was measured
        const std::optional<Pose> pose = poses.poseAt(p.stamp_us);

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
        const double rx = c * p.x_in - s * p.y_in;
        const double ry = s * p.x_in + c * p.y_in;

        // robot motion from the point's time to the reference, world frame
        const double dx = pose->x - ref.x;
        const double dy = pose->y - ref.y;

        // that motion in the reference robot frame
        const double tx = cRef * dx - sRef * dy;
        const double ty = sRef * dx + cRef * dy;

        // deskewed point, tagged with its sensor
        out.push_back(DeskewedPoint{rx + tx, ry + ty, scan.sensor_id});
    }
    return dropped;
}

void ScanMerger::addScan(const RobotScan& scan) {
    // drop an empty scan or unknown sensor
    if (scan.points.empty() || scan.sensor_id >= LIDAR_NUM_SENSORS) {
        return;
    }

    // driver threads write, build() reads
    std::lock_guard<std::mutex> lock(m_mutex);

    // drop sweep that isn't newer than the stored one
    const std::optional<RobotScan>& stored = m_latest[scan.sensor_id];
    if (stored.has_value() && scan.t_end_us <= stored->t_end_us) {
        return;
    }

    // keep this sensor's newest sweep
    m_latest[scan.sensor_id] = scan;
}

std::optional<DeskewedCloud> ScanMerger::build(const PoseBuffer& poses) {
    // TODO: lock, copy out m_latest, unlock
    // TODO: no sweeps -> nullopt
    // TODO: tRef = max t_end_us, tRef <= m_lastRefUs -> nullopt
    // TODO: skip a sensor with no sweep or tRef - t_end_us > LIDAR_STALE_SWEEPS * LIDAR_SWEEP_US,
    //       set singleSensor
    // TODO: ref = poses.poseAt(tRef), nullopt -> nullopt without touching m_lastRefUs
    // TODO: deskewScan each fresh sweep into one cloud, sum dropped
    // TODO: m_lastRefUs = tRef, return the cloud
    (void)poses;
    return std::nullopt;
}
