#pragma once

#include "daydream/subsystems/subsystem.hpp"

#include "api.h"

namespace daydream {

class Drivetrain : public Subsystem {
public:
    Drivetrain(
        pros::Controller& controller,
        pros::MotorGroup& leftMotors,
        pros::MotorGroup& rightMotors);

    void initialize() override;
    void control() override;

    void setVelocity(double left, double right);
    void setBrakeMode(pros::motor_brake_mode_e_t mode);
    void stop();

private:
    pros::Controller& m_controller;
    pros::MotorGroup& m_leftMotors;
    pros::MotorGroup& m_rightMotors;
};

} // namespace daydream
