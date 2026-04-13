#pragma once

#include <vector>
#include "odometry.hpp"
#include "helpers.hpp"
#include "pid.hpp"

constexpr double PurPur_KP = 1.0;
constexpr double PurPur_KI = 0.0;
constexpr double PurPur_KD = 0.0;

constexpr double TURN_RATE = 9; // hi 20

constexpr int MAX_VIEWABLE_INDEX_AHEAD = 10;
constexpr double CURV_SPEED_ADJUSTMENT = 1;
constexpr int MIN_BASE_VEL = 15;
constexpr int MAX_BASE_VEL = 75;

constexpr double END_TOLERANCE = 3;

class PurePursuit {
    private:
        void updateClosestPointIndex(Position cur_position);
        Position getLookaheadPoint(Position cur_position);
        Position convertToRobotCoords(Position robot_pos, double robot_heading_deg, Position target_point);
        double getLookaheadDist();
        double getCurvature(Position pt1, Position pt2);
        
        double wheelbase; // may be useful but idk
        PID velocityPID;
        
        std::vector<Position> path;
        double look_ahead_dist = 10.0; 
        
        int last_passed_point_index = 0;
        int closest_pt_idx = 0;
        
    public:
        PurePursuit(std::vector<Position> path);

        void setPath(std::vector<Position> new_path);

        bool step();
};