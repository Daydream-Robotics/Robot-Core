#pragma once

#include "arclengthSplining.hpp"
#include "odometry.hpp"
#include <numbers>


//STANLEY CONSTANTS
constexpr double STANLEY_K = 1.5;
constexpr double STANLEY_K_SOFT = 5.0;
constexpr double STANLEY_MAX_STEER = (std::numbers::pi/180) * 60;

//END CONDITIONS
constexpr double STANLEY_END_TOL = 1.0;
constexpr double STANLEY_END_SLOW = 5.0;
constexpr int STANLEY_MAX_VEL = 400;
constexpr int STANLEY_MIN_VEL = 50;
constexpr double STANLEY_TURN_SCALE = 150.0;

class StanleyController {
    public:
        StanleyController() = default;

        void setPath(ALS_Path& path);

        bool step(int direction = 1);

    private:
        ALS_Path* m_als_path;
        std::size_t m_closestIdx = 0;
};