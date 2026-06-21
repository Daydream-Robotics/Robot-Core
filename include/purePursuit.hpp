#pragma once

#include <vector>
#include "odometry.hpp"
#include "helpers.hpp"
#include "arclengthSplining.hpp"
#include "motionController.hpp"

constexpr double MAX_LOOKAHEAD_DIST = 25.0;     /**< Maximum lookahead distance (inches) */
constexpr double MIN_LOOKAHEAD_DIST = 15.0;     /**< Minimum lookahead distance (inches) */
constexpr double LOOKAHEAD_SECONDS = 0.6;       /**< How far ahead the robot looks in seconds based on the current linear velocity */
constexpr double TURN_RATE = 9;                 /**< Scaling factor to convert curvature into a turn speed */

constexpr double SPEED_ADJUSTMENT_CONST = 12;   /**< Controls how much the robot slows down on sharp curves */
constexpr int MIN_BASE_VEL = 50;                /**< Mimimum motor speed allowed (RPM) */
constexpr int MAX_BASE_VEL = 350;               /**< Maximum motor speed allowed (RPM) */

constexpr double END_SLOWDOWN_THRESH = 20.0;    /**< Distance from the end of the path to start decelerating (inches) */
constexpr double END_GHOST_CAST = 20.0;         /**< Distance to project ghost target point past end of path */

/**
 * @class PurePursuitController
 * @brief Path tracker that steers the robot along a path by chasing a lookahead point
 */
class PurePursuitController : public MotionController {
    private:
        /**
         * @brief Calculates the curvature of an arc from the robot's center to the target point
         * @param robotFrameTargetPt The target lookahead point in the local robot coordinate frame.
         * @returns Curvature (1 / radius)
         */
        double calculateCurvature(Position robotFrameTargetPt);

        /**
         * @brief Translates the globabl field cooridnate point to the robot's local frame
         * @param targetPoint Global X, Y point to convert
         * @param currentPose Current global pose of the robot
         * @returns Localized Position struct
         */
        Position convertPtToRobotFrame(Position targetPoint, const Pose& currentPose);

        /**
         * @brief Calculates the dynamic lookahead distance based off the robot's speed
         * @returns Lookahead distance (inches)
         */
        double getLookaheadDist();

        /**
         * @brief Calculates the base velocity based off the curvature to the target point and the distancce to the end of the path
         * @note slows for sharp curves and end of path
         * @param curvature Cacluated curvature to the target point
         * @returns Motor base velocity (RPM)
         */
        int getBaseVelocity(double curvature);

        double m_lookAheadDist = 10.0;      /**< The current loop's lookahead distance */
        int m_stepCounter = 0;              /**< The number of times step has been called */

        // modifiable
        double m_speedMultiplier = 1;       /**< Scalar to adjust overall speed */ // TODO: make configurable from main or path follower
        
    public:
        PurePursuitController();
        virtual ~PurePursuitController() = default;

        /**
         * @brief Computes the commanded wheel velocities
         * @param currentPose latest pose of the robot
         * @param als_path splined path with target samples
         * @param closestSampleIdx lookup index of the closest sample to the robot
         * @param flag sets direction of tracking
         * @returns left and right wheel velocities 
         */
        WheelVelocities compute(const Pose& currentPose, const ALS_Path& als_path, std::size_t& closestSampleIdx, PathFlag flag) override;

        /**
         * @brief resets tracking variables to inital values
         */
        void reset() override;

        double m_totalDistOff = 0;      /**< Accumulated totoal cross-track error over the course of the path */
        double m_distFromEnd;           /**< Arc length distance remaining till the end of the path */
};
