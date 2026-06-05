#pragma once
#include "arclengthSplining.hpp"
#include "odometry.hpp"

struct WheelVelocities {
    double left;
    double right;
};

class MotionController {
    public:
        virtual ~MotionController() = default;
        virtual WheelVelocities compute(const Pose& currentPose, const Sample& targetSample) = 0;
};