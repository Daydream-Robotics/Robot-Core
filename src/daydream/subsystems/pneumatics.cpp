/**
 * @file pneumatics.cpp
 * @brief Project interface or implementation.
 */

#include "daydream/subsystems/pneumatics.hpp"

namespace daydream {

Pneumatics::Pneumatics(pros::adi::Pneumatics& ballBlocker,
                       pros::adi::Pneumatics& scoringLifter,
                       pros::adi::Pneumatics& matchloader,
                       pros::adi::Pneumatics& descorer)
	: m_ballBlocker(ballBlocker),
	  m_scoringLifter(scoringLifter),
	  m_matchloader(matchloader),
	  m_descorer(descorer) {}

void Pneumatics::initialize() {}

void Pneumatics::updateFromController(pros::Controller& controller) {
	static_cast<void>(controller);
	/// Add driver input handling here.
}

void Pneumatics::setBallBlocker(bool extended) {
	m_ballBlocker.set_value(extended);
}

void Pneumatics::setScoringLifter(bool extended) {
	m_scoringLifter.set_value(extended);
}

void Pneumatics::setMatchloader(bool extended) {
	m_matchloader.set_value(extended);
}

void Pneumatics::setDescorer(bool extended) {
	m_descorer.set_value(extended);
}

}  // namespace daydream
