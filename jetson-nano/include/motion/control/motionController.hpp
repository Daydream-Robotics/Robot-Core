#pragma once
#include "motion/pathing/arclengthSplining.hpp"
#include "localization/odometry.hpp"

/**
 * @brief Configuration modes for drivetrain motor inputs
 */
enum class ControlMode {
    INPUT_VELOCITY,
    INPUT_VOLTAGE
};

/**
 * @brief Direction flags for paths
 */
enum class PathFlag {
    FORWARDS,
    REVERSE
};

/**
 * @struct WheelVelocities
 * @brief output command containing velocity tagerts for both left and right drive motors
 * @note INPUT_VELOCITY: rpm at the motor output shaft. INPUT_VOLTAGE: mV, -12000..12000
 */
struct WheelVelocities {
    double left; /**< command value for left motor group */
    double right; /**< command value for right motor group */
    //! IMPORTANT: Would have renamed WheelVelocities but didn't want to make merging everyone else's code too difficult
    ControlMode input = ControlMode::INPUT_VELOCITY;
};

/**
 * @struct DriveState
 * @brief drivetrain measurements from the brain's latest STATE
 */
struct DriveState {
    double omegaLeft; /**< left wheel speed in rad/s (wheel side, after external gearing) */
    double omegaRight; /**< right wheel speed in rad/s (wheel side, after external gearing) */
    double batteryVolts; /**< battery voltage measured under load in volts */
};

/**
 * @class MotionController
 * @brief Abstract parent class for h-drive chassis movemnent controllers
 */
class MotionController {
    public:
        virtual ~MotionController() = default;

        /**
         * @brief Computes the commanded wheel velocities
         * @param currentPose latest pose of the robot
         * @param drive latest drivetrain measurements
         * @param als_path splined path with target samples
         * @param closestSampleIdx lookup index of the closest sample to the robot
         * @param flag sets direction of tracking
         * @returns left and right wheel velocities 
         */
        virtual WheelVelocities compute(const Pose& currentPose, const DriveState& drive, const ALS_Path& als_path, std::size_t& closestSampleIdx, PathFlag flag) = 0;
        
        /**
         * @brief resets tracking variables to inital values
         * @note This implementaion is optional for child classes
         */
        virtual void reset() {}
};
