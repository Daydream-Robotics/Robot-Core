#include "daydream/subsystems/pneumatics.hpp"

namespace daydream {

Pneumatics::Pneumatics(
    pros::Controller& controller,
    pros::adi::Pneumatics& ballBlocker,
    pros::adi::Pneumatics& scoringLifter,
    pros::adi::Pneumatics& matchloader,
    pros::adi::Pneumatics& descorer)
    : m_controller(controller),
      m_ballBlocker(ballBlocker),
      m_scoringLifter(scoringLifter),
      m_matchloader(matchloader),
      m_descorer(descorer) {}

void Pneumatics::initialize() {
    setBallBlocker(false);
    setScoringLifter(false);
    setMatchloader(false);
    setDescorer(false);
}

void Pneumatics::control() {
    while (!pros::competition::is_disabled() && !pros::competition::is_autonomous()) {
        setBallBlocker(m_controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2));
        setMatchloader(m_controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y));
        setDescorer(!m_controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1));

        if (m_controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            setScoringLifter(!m_scoringLifterRaised);
        }

        pros::delay(20);
    }
}

void Pneumatics::setBallBlocker(bool extended) {
    m_ballBlocker.set_value(extended);
}

void Pneumatics::setScoringLifter(bool raised) {
    m_scoringLifterRaised = raised;
    m_scoringLifter.set_value(raised);
}

void Pneumatics::setMatchloader(bool extended) {
    m_matchloader.set_value(extended);
}

void Pneumatics::setDescorer(bool extended) {
    m_descorer.set_value(extended);
}

} // namespace daydream
