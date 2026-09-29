/**
 * @file drivetrain.hpp
 * @brief Project interface or implementation.
 */

#pragma once

#include <cstdint>

#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

namespace daydream {

/**
 * @brief Owns the drivetrain interface used by robot modes and autonomous routines.
 * @details The motor groups are supplied by the application and retained by
 * reference, so all modes operate on the same hardware objects.
 */
class Drivetrain {
public:
	/** @brief Binds this subsystem to the configured left and right motor groups. */
	Drivetrain(pros::MotorGroup& leftMotors, pros::MotorGroup& rightMotors);

	/** @brief Applies subsystem startup configuration. */
	void initialize();
	/** @brief Reads driver controls and updates the drivetrain command. */
	void driveFromController(pros::Controller& controller);
	/** @brief Commands left and right wheel group velocities. */
	void setWheelVelocity(double leftVelocity, double rightVelocity);
	/** @brief Commands left and right wheel group voltages in millivolts. */
	void setWheelVoltage(std::int32_t leftMillivolts, std::int32_t rightMillivolts);
	/** @brief Stops both motor groups. */
	void stop();

private:
	pros::MotorGroup& m_leftMotors;
	pros::MotorGroup& m_rightMotors;
};

}  // namespace daydream
