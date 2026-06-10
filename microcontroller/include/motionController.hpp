#pragma once
#include "arclengthSplining.hpp"
#include "odometry.hpp"

enum class ControlMode {
    INPUT_VELOCITY,
    INPUT_VOLTAGE
};

struct WheelVelocities {
    double left;
    double right;
    //! IMPORTANT: Would have renamed WheelVelocities but didn't want to make merging everyone else's code too difficult
    ControlMode input = ControlMode::INPUT_VELOCITY;
};

class MotionController {
    public:
        virtual ~MotionController() = default;
        virtual WheelVelocities compute(const Pose& currentPose, const ALS_Path& als_path, std::size_t& closestSampleIdx) = 0;
        
        /**
         * @brief resets tracking variables to inital values
         * @note This implementaion is optional for child classes
         */
        virtual void reset() {}
};