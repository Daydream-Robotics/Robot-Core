#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <optional>

#include "config/constants.h"
#include "localization/ekf/ekfInterface.hpp"
#include "localization/odometry.hpp"

/**
 * @brief Recent odom_ekf_ poses, interpolated at any time in the last second.
 *
 * One writer (the odometry thread) and any number of readers (the LiDAR threads).
 */
class PoseBuffer {
public:
    /**
     * @brief Adds the newest pose.
     *
     * @param tUs Time of the pose, Nano steady_clock (us).
     * @param pose odom_ekf_ pose at tUs.
     */
    void push(uint64_t tUs, const Pose& pose);

    /**
     * @brief Pose at time t, linearly interpolated between the two samples around it.
     *
     * @param tUs Query time (us).
     * @return std::optional<Pose> Nothing if t is older than the buffer, or newer than the
     * newest sample by more than LIDAR_POSE_EXTRAPOLATE_US.
     */
    std::optional<Pose> poseAt(uint64_t tUs) const;

    /**
     * @brief Clears every sample.
     */
    void clear();

    /**
     * @brief Pushes every odom_ekf_ pose from ekf into this buffer. Call once.
     *
     * @warning The buffer must outlive ekf's callbacks, the callback holds a pointer to this buffer.
     *
     * @param ekf EKF to listen to.
     */
    void subscribe(EkfInterface& ekf);

private:
    /**
     * @brief One stored pose.
     */
    struct Sample {
        uint64_t tUs = 0; ///< Nano steady_clock time (us)
        Pose pose{0.0, 0.0, 0.0}; ///< odom_ekf_ pose at tUs
    };

    /**
     * @brief Blends two samples, heading wrapped.
     *
     * @param a Earlier sample.
     * @param b Later sample.
     * @param alpha Fraction of the way from a to b, 0 to 1.
     * @return Pose Interpolated pose.
     */
    static Pose interpolate(const Sample& a, const Sample& b, double alpha);

    mutable std::mutex m_mutex; ///< guards the ring buffer
    std::array<Sample, LIDAR_POSE_BUFFER_CAPACITY> m_samples{}; ///< ring buffer, oldest overwritten first
    int m_head = 0; ///< index the next sample is written to
    int m_count = 0; ///< how many of m_samples are filled
    bool m_lowRateLogged = false; ///< low rate already logged
};
