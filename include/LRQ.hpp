#pragma once
#include "constants.h"
#include "odometry.hpp"
#include "motionController.hpp"

struct LQRConfig {
    double qX = 1.0; // State error penalty for longitudinal position
    double qY = 6.0; // State error penalty for lateral position
    double qTheta = 2.0; // State error penalty for heading
    double rV = 1.0; // Control effort penalty for linear velocity
    double rOmega = 1.0; // Control effort penalty for angular velocity
    double dt = 0.01; // Time step in seconds
    double trackWidthInches = DRIVE_TRACK_WIDTH_INCHES; // Track width of the robot in inches
};

class LQRController : public MotionController {
    public:
        explicit LQRController(LQRConfig config);

        WheelVelocities compute(const Pose& currentPose, const ALS_Path& als_path, std::size_t& closestSampleIdx, PathFlag flag) override;

        void reset() override;
    
    private:
        LQRConfig m_config;
};

