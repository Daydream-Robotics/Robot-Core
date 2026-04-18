#pragma once

#include <vector>
#include "odometry.hpp"
#include "helpers.hpp"
#include "pid.hpp"
#include "arclengthSplining.hpp"

constexpr double PurPur_KP = 1.0; // unused
constexpr double PurPur_KI = 0.0; // unused
constexpr double PurPur_KD = 0.0; // unused

constexpr double MAX_LOOKAHEAD_DIST = 25.0;
constexpr double MIN_LOOKAHEAD_DIST = 18.0;
constexpr double LOOKAHEAD_SECONDS = 0.6; // this it the amount of time the robot looks ahead of it for pure pursuit

constexpr double TURN_RATE = 5; // Moderated to prevent aggressive oscillation

// constexpr int MAX_VIEWABLE_INDEX_AHEAD = 10;

constexpr double SPEED_ADJUSTMENT_CONST = 10.0; // Reduced to prevent the robot from over-braking on curves
constexpr int MIN_BASE_VEL = 10;
constexpr int MAX_BASE_VEL = 100;

constexpr double END_TOLERANCE = 3;
constexpr double END_SLOWDOWN_THRESH = 20.0;

constexpr double END_GHOST_CAST = 20.0;

class PurePursuit {
    private:
        // calculates the curvature of a point within the robot frame
        double calculateCurvature(Position robotFrameTargetPt);

        // converts a target point to a point local to the robot's frame of reference
        Position convertPtToRobotFrame(Position targetPoint);

        // returns the dynamic lookahead distance adjusted for the robot speed
        double getLookaheadDist();

        // returns base velocity based off curvature to target point and distance to end of path
        int getBaseVelocity(double curvature);
            
        // update ghost point to cast past end point
        Position updateGhostPoint();

        PID m_velocityPID;

        ALS_Path& m_als_path;

        Position m_ghostPoint;
        double m_lookAheadDist = 10.0;
        int m_lastPassedPtIdx = 0;
        int m_stepCounter = 0;
        
    public:
        PurePursuit(ALS_Path& als_path);

        void setPath(std::vector<Position> new_path);

        bool step();

        double m_totalDistOff = 0;
        double m_distFromEnd;
};