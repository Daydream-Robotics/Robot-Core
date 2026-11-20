#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <optional>
#include <vector>

#include "config/constants.h"
#include "localization/odometry.hpp"
#include "perception/lidar/lidarTypes.hpp"
#include "perception/lidar/poseBuffer.hpp"

/**
 * @brief One deskewed point in the robot frame at the cloud's reference time.
 */
struct DeskewedPoint {
    double xIn = 0.0; ///< robot frame at t_ref, +X forward (in)
    double yIn = 0.0; ///< robot frame at t_ref, +Y left (in)
    uint8_t sensorId = 0; ///< sensor that saw the point
};

/**
 * @brief Both sensors' latest sweeps, deskewed to one reference time.
 */
struct DeskewedCloud {
    uint64_t tRefUs = 0; ///< every point is expressed at this time (us)
    std::vector<DeskewedPoint> points; ///< both sensors, merged
    uint64_t dropped = 0; ///< points with no pose in the buffer
    bool singleSensor = false; ///< one sensor was stale or missing
};

/**
 * @brief Keeps each sensor's latest sweep and merges them into one deskewed cloud.
 *
 * addScan() is called from the driver threads, build() from the registration thread.
 */
class ScanMerger {
public:
    /**
     * @brief Stores a sensor's newest complete sweep.
     *
     * @param scan Robot-frame scan. Ignored if empty or the sensorId is out of range.
     */
    void addScan(const RobotScan& scan);

    /**
     * @brief Deskews the latest sweeps to t_ref = newest sweep end.
     *
     * @param poses odom_ekf_ pose buffer.
     * @return std::optional<DeskewedCloud> Nothing if there is no new sweep since the last
     * cloud, or no pose at t_ref yet.
     */
    std::optional<DeskewedCloud> build(const PoseBuffer& poses);

    /**
     * @brief Deskews one scan into the robot frame at the reference pose.
     *
     * p* = R(theta_i - theta_ref) * p + R(-theta_ref) * (t_i - t_ref), using odom_ekf_ poses only.
     *
     * @param scan Robot-frame scan from one sensor.
     * @param poses odom_ekf_ pose buffer.
     * @param ref odom_ekf_ pose at the reference time.
     * @param out Deskewed points are appended here.
     * @return uint64_t Points dropped because poseAt() had no pose.
     */
    static uint64_t deskewScan(const RobotScan& scan, const PoseBuffer& poses, const Pose& ref,
                               std::vector<DeskewedPoint>& out);

private:
    mutable std::mutex m_mutex; ///< guards m_latest
    std::array<std::optional<RobotScan>, LIDAR_NUM_SENSORS> m_latest{}; ///< newest sweep per sensor
    uint64_t m_lastRefUs = 0; ///< t_ref of the last built cloud (us)
};
