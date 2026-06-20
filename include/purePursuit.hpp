#pragma once

#include <vector>
#include "odometry.hpp"
#include "helpers.hpp"
#include "arclengthSplining.hpp"
#include "motionController.hpp"
#include "pathFlags.hpp"

constexpr double MAX_LOOKAHEAD_DIST = 25.0;
constexpr double MIN_LOOKAHEAD_DIST = 15.0;
constexpr double LOOKAHEAD_SECONDS = 0.6; // 0.6 this it the amount of time the robot looks ahead of it for pure pursuit

constexpr double TURN_RATE = 9;  // 9(0.1477) 10(0.1154) 11(0.1136) 12(0.1269) Moderated to prevent aggressive oscillation

// constexpr int MAX_VIEWABLE_INDEX_AHEAD = 10;

constexpr double SPEED_ADJUSTMENT_CONST = 12; // 4.5) Reduced to prevent the robot from over-braking on curves
constexpr int MIN_BASE_VEL = 50;
constexpr int MAX_BASE_VEL = 350;

constexpr double END_SLOWDOWN_THRESH = 20.0; // 20

constexpr double END_GHOST_CAST = 20.0;

class PurePursuitController : public MotionController {
    private:
        // calculates the curvature of a point within the robot frame
        double calculateCurvature(Position robotFrameTargetPt);

        // converts a target point to a point local to the robot's frame of reference
        Position convertPtToRobotFrame(Position targetPoint, const Pose& currentPose);

        // returns the dynamic lookahead distance adjusted for the robot speed
        double getLookaheadDist();

        // returns base velocity based off curvature to target point and distance to end of path
        int getBaseVelocity(double curvature);

        double m_lookAheadDist = 10.0;
        int m_stepCounter = 0;

        // modifiable
        double m_speedMultiplier = 1;
        
    public:
        PurePursuitController();
        virtual ~PurePursuitController() = default;

        WheelVelocities compute(const Pose& currentPose, const ALS_Path& als_path, std::size_t& closestSampleIdx, PathFlag flag) override;

        void reset() override;

        double m_totalDistOff = 0;
        double m_distFromEnd;
};
