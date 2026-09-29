/**
 * @file drivetrain.cpp
 * @brief Project interface or implementation.
 */

#include "daydream/subsystems/drivetrain.hpp"

namespace daydream {

Drivetrain::Drivetrain(pros::MotorGroup& leftMotors, pros::MotorGroup& rightMotors)
	: m_leftMotors(leftMotors), m_rightMotors(rightMotors) {}

void Drivetrain::initialize() {}

void Drivetrain::driveFromController(pros::Controller& controller) {
	static_cast<void>(controller);
	/// Add the driver-control mapping here.
}

void Drivetrain::setWheelVelocity(double leftVelocity, double rightVelocity) {
	m_leftMotors.move_velocity(leftVelocity);
	m_rightMotors.move_velocity(rightVelocity);
}

void Drivetrain::setWheelVoltage(std::int32_t leftMillivolts, std::int32_t rightMillivolts) {
	m_leftMotors.move_voltage(leftMillivolts);
	m_rightMotors.move_voltage(rightMillivolts);
}

void Drivetrain::stop() {
	setWheelVoltage(0, 0);
}

}  // namespace daydream
