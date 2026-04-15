#pragma once

#include <vector>
#include "odometry.hpp"
#include "helpers.hpp"
#include "pid.hpp"
#include "arclengthSplining.hpp"

constexpr double PurPur_KP = 1.0; // unused
constexpr double PurPur_KI = 0.0; // unused
constexpr double PurPur_KD = 0.0; // unused

constexpr double MAX_LOOKAHEAD_DIST = 20.0;
constexpr double MIN_LOOKAHEAD_DIST = 7.0;
constexpr double LOOKAHEAD_SECONDS = 1; // this it the amount of time the robot looks ahead of it for pure pursuit

constexpr double TURN_RATE = 2.5; // Too high can stall the inner wheel on sharp turns

constexpr int MAX_VIEWABLE_INDEX_AHEAD = 10;

constexpr double SPEED_ADJUSTMENT_CONST = 7.0; // Too high can cause the robot to overbrake on curves
constexpr int MIN_BASE_VEL = 40;
constexpr int MAX_BASE_VEL = 100;

constexpr double END_TOLERANCE = 3;

class PurePursuit {
    private:
        // calculates the curvature of a point within the robot frame
        double calculateCurvature(Position robotFrameTargetPt);

        // converts a target point to a point local to the robot's frame of reference
        Position convertPtToRobotFrame(Position targetPoint);

        // returns the dynamic lookahead distance adjusted for the robot speed
        double getLookaheadDist();

        // returns the index of the point along the path that is currently closest to the robot.
        int getClosestPtIdx(Position robotPosition);

        // returns base velocity based off curvature to target point and distance to end of path
        int getBaseVelocity(double curvature);
        
        // DEPRICATED
        Position getLookaheadPoint(Position currentPosition, int closestPtIdx);
    
        PID velocityPID;
         
        std::vector<Position> path;
        ALS_Path& als_path;

        double lookAheadDist = 10.0; 
        
        int lastPassedPtIdx = 0;

        int stepCounter = 0;
        
    public:
        PurePursuit(std::vector<Position> path, ALS_Path& als_path);

        void setPath(std::vector<Position> new_path);

        bool step();
};