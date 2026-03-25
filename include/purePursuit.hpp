#pragma once

#include <vector>
#include "odometry.hpp"
#include "helpers.hpp"
#include "pid.hpp"

constexpr double PP_KP = 1.0;
constexpr double PP_KI = 0.0;
constexpr double PP_KD = 0.0;

struct TargetPoint : Position {
};

class PurePursuit {
    private:
        std::vector<TargetPoint> path;
        
        int findClosestPoint(Position position);
        Position findLookaheadPoint(Position position);
        
        double wheelbase;
        PID velocityPID;
        double look_ahead_dist;


    public:
        PurePursuit(double wheelBase);

        void setPath(std::vector<TargetPoint> new_path);

        std::pair<double, double> step();
};