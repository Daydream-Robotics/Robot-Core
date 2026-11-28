#pragma once
#include <array>
#include "motion/pathing/arclengthSplining.hpp"
#include "localization/odometry.hpp"
#include "mpc/drivetrain.hpp"

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
 * @struct DriveCommand
 * @brief output command with one value per drive motor group
 * @note groups are in the drivetrain's group order (see mpc/drivetrain.hpp), entries past its group count are unused.
 *       INPUT_VELOCITY: rpm at the motor output shaft. INPUT_VOLTAGE: mV, -12000..12000
 */
struct DriveCommand {
    std::array<double, kinematics::MAX_MOTOR_GROUPS> value{}; /**< command value per motor group */
    ControlMode input = ControlMode::INPUT_VELOCITY;
};

/**
 * @struct DriveState
 * @brief drivetrain measurements from the brain's latest STATE
 */
struct DriveState {
    std::array<double, kinematics::MAX_MOTOR_GROUPS> omega{}; /**< wheel speed per motor group in rad/s (wheel side, after external gearing), drivetrain group order */
    double batteryVolts = 0.0; /**< battery voltage measured under load in volts */
    double age = 0.0; /**< seconds since the brain measured the wheel speeds */
};

/**
 * @class MotionController
 * @brief Abstract parent class for chassis movement controllers (diff, X or H drive)
 */
class MotionController {
    public:
        virtual ~MotionController() = default;

        /**
         * @brief Computes the drive command
         * @param currentPose latest pose of the robot
         * @param drive latest drivetrain measurements
         * @param als_path splined path with target samples
         * @param closestSampleIdx lookup index of the closest sample to the robot
         * @param flag sets direction of tracking
         * @returns one command per drive motor group
         */
        virtual DriveCommand compute(const Pose& currentPose, const DriveState& drive, const ALS_Path& als_path, std::size_t& closestSampleIdx, PathFlag flag) = 0;

        /**
         * @brief moves the measured pose and wheel speeds forward to when the next command will apply
         * @param pose robot pose, advanced in place
         * @param drive drivetrain measurements, advanced in place (age becomes 0)
         * @note optional for child classes: by default the measurements are used as they are
         */
        virtual void predict(Pose& pose, DriveState& drive) const {
            (void)pose;
            (void)drive;
        }

        /**
         * @brief resets tracking variables to inital values
         * @note This implementaion is optional for child classes
         */
        virtual void reset() {}
};
