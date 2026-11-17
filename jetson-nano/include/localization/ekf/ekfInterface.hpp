#pragma once

#include "localization/odometry.hpp"

#include <cstdint>
#include <functional>
#include <optional>
#include <Eigen/Dense>

struct StampedPose {
    uint64_t stamp_us = 0;
    Pose pose{0.0, 0.0, 0.0};
    std::optional<Eigen::Matrix3d> cov = std::nullopt;
};


struct LidarMeasurement {
    uint64_t t_ref_us;
    Pose pose;
    Eigen::Matrix3d cov;
    uint32_t health_flags;
};


class EkfInterface {
public:
    virtual ~EkfInterface() = default;

    virtual void onOdomPose(std::function<void(const StampedPose&)> cb) = 0;

    virtual std::optional<StampedPose> mainPoseAt(uint64_t t_us) = 0;

    virtual StampedPose mainPoseLatest() = 0;

    virtual void submitLidarMeasurement(const LidarMeasurement& m) = 0;

    virtual void onPoseReset(std::function<void(const StampedPose&)> cb) = 0;

};
