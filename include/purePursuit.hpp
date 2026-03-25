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
        Position findClosestPoint(Position position);
        Position findLookaheadPoint(Position position);
        
        double wheelbase; // may be useful but idk
        PID velocityPID;
        
        std::vector<Position> path;
        int lastPassedPointIndex = 0;
        double look_ahead_dist;

    public:
        PurePursuit(std::vector<Position> path);

        void setPath(std::vector<TargetPoint> new_path);

        std::pair<double, double> step();
};