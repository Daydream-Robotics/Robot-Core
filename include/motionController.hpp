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
        virtual WheelVelocities compute(const Pose& currentPose, const ALS_Path& als_path, std::size_t& closestSampleIdx) = 0;
        
        /**
         * @brief resets tracking variables to inital values
         * @note This implementaion is optional for child classes
         */
        virtual void reset() {}
};