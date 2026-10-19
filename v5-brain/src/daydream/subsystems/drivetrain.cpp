#include "daydream/subsystems/drivetrain.hpp"

#include "daydream/config/constants.h"

#include <cstdlib>

namespace daydream {

Drivetrain::Drivetrain(
    pros::Controller& controller,
    pros::MotorGroup& leftMotors,
    pros::MotorGroup& rightMotors)
    : m_controller(controller),
      m_leftMotors(leftMotors),
      m_rightMotors(rightMotors) {}

void Drivetrain::initialize() {
    m_leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    m_rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
}

void Drivetrain::control() {
    setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    while (!pros::competition::is_disabled() && !pros::competition::is_autonomous()) {
        const int forward = m_controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        const int turn = m_controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        const int left = forward + turn;
        const int right = forward - turn;

        m_leftMotors.move_voltage(std::abs(left) > DEADZONE ? left * 12000 / 127 : 0);
        m_rightMotors.move_voltage(std::abs(right) > DEADZONE ? right * 12000 / 127 : 0);

        pros::delay(20);
    }

    stop();
}

void Drivetrain::setVelocity(double left, double right) {
    m_leftMotors.move_velocity(left);
    m_rightMotors.move_velocity(right);
}

void Drivetrain::setBrakeMode(pros::motor_brake_mode_e_t mode) {
    m_leftMotors.set_brake_mode_all(mode);
    m_rightMotors.set_brake_mode_all(mode);
}

void Drivetrain::stop() {
    setVelocity(0.0, 0.0);
}

} // namespace daydream
