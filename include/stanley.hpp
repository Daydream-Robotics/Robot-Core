#pragma once

#include "arclengthSplining.hpp"
#include "odometry.hpp"
#include <numbers>


//STANLEY CONSTANTS
constexpr double STANLEY_K = 4.0; //tuning constant
constexpr double STANLEY_K_SOFT = 0.5; //Soft K value to prevent divide by zero errors
constexpr double STANLEY_TURN_SCALE = 150.0;
constexpr double STANLEY_MAX_STEER = (std::numbers::pi/180) * 90; //converts degrees to radians for the ease of the brain

//END CONDITIONS
constexpr double STANLEY_END_TOL = 1.5; //distance from end of path
constexpr double STANLEY_END_SLOW = 12.0; // slowing down at end of path to help prevent sudden stops and overshoots


//VELECITY CONSTRAINTS
constexpr int STANLEY_MAX_VEL = 275;
constexpr int STANLEY_MIN_VEL = 40;




//Stanley Class
class StanleyController {
    public:
        StanleyController() = default;

        void setPath(ALS_Path& path);
        
        bool step(int direction = 1);

    private:
        ALS_Path* m_als_path;
        std::size_t m_closestIdx = 0;
};