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
constexpr double STANDLEY_END_SLOW = 12.0;
constexpr int STANLEY_MAX_VEL = 200;
constexpr int STANLEY_MIN_VEL = 40;
constexpr double STANLEY_TURN_SCALE = 150.0;

class StanleyController {
    public:
        StanleyController() = default;

        void setPath(ALS_Path& path);
        /*direction of 1 goes forward
          direction of -1 goes backwards
        */
        bool step();

    private:
        ALS_Path* m_als_path;
        std::size_t m_closestIdx = 0;
};