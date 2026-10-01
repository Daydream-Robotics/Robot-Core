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

private:
    pros::Controller& m_controller;
    pros::MotorGroup& m_leftMotors;
    pros::MotorGroup& m_rightMotors;
};

} // namespace daydream
