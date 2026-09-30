#pragma once

#include <vector>

#include "daydream/motion/odometry.hpp"
#include "daydream/utils/helpers.hpp"
#include "daydream/motion/pathing/arclengthSplining.hpp"
#include "daydream/motion/control/motionController.hpp"

constexpr double MAX_LOOKAHEAD_DIST = 25.0;     /**< Maximum lookahead distance (inches). */
constexpr double MIN_LOOKAHEAD_DIST = 15.0;     /**< Minimum lookahead distance (inches). */
constexpr double LOOKAHEAD_SECONDS = 0.6;       /**< How far ahead the robot looks in seconds based on its current linear velocity. */
constexpr double TURN_RATE = 9;                 /**< Scaling factor used to convert curvature into turn speed. */

constexpr double SPEED_ADJUSTMENT_CONST = 12;   /**< Controls how much the robot slows down on sharp curves. */
constexpr int MIN_BASE_VEL = 50;                /**< Minimum motor speed allowed (RPM). */
constexpr int MAX_BASE_VEL = 350;               /**< Maximum motor speed allowed (RPM). */

constexpr double END_SLOWDOWN_THRESH = 20.0;    /**< Distance from the end of the path at which to start decelerating (inches). */
constexpr double END_GHOST_CAST = 20.0;         /**< Distance to project the ghost target point past the end of the path. */

/**
 * @class PurePursuitController
 * @brief Path tracker that steers the robot along a path by chasing a lookahead point.
 */
class PurePursuitController : public MotionController {
private:
    /**
     * @brief Calculates the curvature of an arc from the robot's center to the target point.
     *
     * @param robotFrameTargetPt Target lookahead point in the local robot coordinate frame.
     * @return Curvature (1 / radius).
     */
    double calculateCurvature(Position robotFrameTargetPt);

    /**
     * @brief Converts a global field coordinate point to the robot's local coordinate frame.
     *
     * @param targetPoint Global X, Y point to convert.
     * @param currentPose Current global pose of the robot.
     * @return Localized Position struct.
     */
    Position convertPtToRobotFrame(Position targetPoint, const Pose& currentPose);

    /**
     * @brief Calculates the dynamic lookahead distance based on the robot's speed.
     *
     * @return Lookahead distance (inches).
     */
    double getLookaheadDist();

    /**
     * @brief Calculates the base velocity based on curvature and distance to the end of the path.
     *
     * Slows the robot down for sharp curves and when approaching the end of the path.
     *
     * @param curvature Calculated curvature to the target point.
     * @return Motor base velocity (RPM).
     */
    int getBaseVelocity(double curvature);

    double m_lookAheadDist = 10.0;  /**< Current loop's lookahead distance. */
    int m_stepCounter = 0;          /**< Number of times step has been called. */

    // Modifiable.
    double m_speedMultiplier = 1;   /**< Scalar used to adjust overall speed. */

public:
    /**
     * @brief Constructs a PurePursuitController.
     */
    PurePursuitController();

    /**
     * @brief Destroys the PurePursuitController.
     */
    virtual ~PurePursuitController() = default;

    /**
     * @brief Computes the commanded wheel velocities.
     *
     * @param currentPose Latest pose of the robot.
     * @param als_path Splined path containing target samples.
     * @param closestSampleIdx Lookup index of the closest sample to the robot.
     * @param flag Sets the direction of tracking.
     * @return Left and right wheel velocities.
     */
    WheelVelocities compute(
        const Pose& currentPose,
        const ALS_Path& als_path,
        std::size_t& closestSampleIdx,
        PathFlag flag
    ) override;

    /**
     * @brief Resets tracking variables to their initial values.
     */
    void reset() override;

    double m_totalDistOff = 0;  /**< Accumulated total cross-track error over the course of the path. */
    double m_distFromEnd;       /**< Arc length distance remaining until the end of the path. */
};
