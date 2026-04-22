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
constexpr double MIN_LOOKAHEAD_DIST = 10.0;
constexpr double LOOKAHEAD_SECONDS = 0.6; // 0.6 this it the amount of time the robot looks ahead of it for pure pursuit

constexpr double TURN_RATE = 16;  // 16 Moderated to prevent aggressive oscillation lo 5

// constexpr int MAX_VIEWABLE_INDEX_AHEAD = 10;

constexpr double SPEED_ADJUSTMENT_CONST = 20; // 10 Reduced to prevent the robot from over-braking on curves
constexpr int MIN_BASE_VEL = 15;
constexpr int MAX_BASE_VEL = 600;

constexpr double END_TOLERANCE = 0.5;
constexpr double END_SLOWDOWN_THRESH = 20.0; // 20

constexpr double END_GHOST_CAST = 20.0;

class PurePursuit {
    private:
        void reset();

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

        ALS_Path* m_als_path;

        Position m_ghostPoint;
        double m_lookAheadDist = 10.0;
        int m_lastPassedPtIdx = 0;
        int m_stepCounter = 0;
        
    public:
        PurePursuit();
        ~PurePursuit();

        void setPath(ALS_Path& als_path);

        bool step(double velocityDirection = 1.0); // Direction = 1 for forward, -1 for reverse

        double m_totalDistOff = 0;
        double m_distFromEnd;
};
