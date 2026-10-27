#pragma once

#include "daydream/subsystems/subsystem.hpp"

#include "api.h"

namespace daydream {

class Pneumatics : public Subsystem {
public:
    Pneumatics(
        pros::Controller& controller,
        pros::adi::Pneumatics& ballBlocker,
        pros::adi::Pneumatics& scoringLifter,
        pros::adi::Pneumatics& matchloader,
        pros::adi::Pneumatics& descorer);

    void initialize() override;
    void control() override;

    void setBallBlocker(bool extended);
    void setScoringLifter(bool raised);
    void setMatchloader(bool extended);
    void setDescorer(bool extended);

private:
    pros::Controller& m_controller;
    pros::adi::Pneumatics& m_ballBlocker;
    pros::adi::Pneumatics& m_scoringLifter;
    pros::adi::Pneumatics& m_matchloader;
    pros::adi::Pneumatics& m_descorer;
    bool m_scoringLifterRaised = false;
};

} // namespace daydream
