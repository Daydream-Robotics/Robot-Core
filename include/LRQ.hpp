#pragma once
#include "constants.h"
#include "odometry.hpp"
#include "motionController.hpp"

struct LQRConfig {
    double qX;
    double qY;
    double qTheta;
    double rV;
    double rOmega;
    double dt;
    double trackWidthInches;
};

class LQRController : public MotionController {
    public:
        explicit LQRController(LQRConfig config);

        WheelVelocities compute(const Pose& currentPose, const ALS_Path& als_path, std::size_t& closestSampleIdx, PathFlag flag) override;

        void reset() override;
    
    private:
        LQRConfig m_config;
};

