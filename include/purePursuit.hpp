#pragma once

#include <vector>
#include "odometry.hpp"
#include "helpers.hpp"
#include "pid.hpp"

constexpr double PurPur_KP = 1.0;
constexpr double PurPur_KI = 0.0;
constexpr double PurPur_KD = 0.0;

constexpr double TURN_RATE = 50;

constexpr double END_TOLERANCE = 1.5;

class PurePursuit {
    private:
        int findClosestPointIndex(Position cur_position);
        Position findLookaheadPoint(Position cur_position);
        Position convertToRobotCoords(Position robot_pos, double robot_heading_deg, Position target_point);
        
        double wheelbase; // may be useful but idk
        PID velocityPID;
        
        std::vector<Position> path;
        int lastPassedPointIndex = 0;
        double look_ahead_dist = 2;

    public:
        PurePursuit(std::vector<Position> path);

        void setPath(std::vector<Position> new_path);

        bool step();
};