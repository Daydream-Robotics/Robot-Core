/**
 * @file pneumatics.hpp
 * @brief Project interface or implementation.
 */

#pragma once

#include "pros/adi.hpp"
#include "pros/misc.hpp"

namespace daydream {

/**
 * @brief Groups the robot's pneumatic actuators behind a shared subsystem interface.
 * @details Hardware objects are injected by the application and retained by
 * reference for use in autonomous and operator control.
 */
class Pneumatics {
public:
	/** @brief Binds the subsystem to the configured pneumatic actuators. */
	Pneumatics(pros::adi::Pneumatics& ballBlocker,
	           pros::adi::Pneumatics& scoringLifter,
	           pros::adi::Pneumatics& matchloader,
	           pros::adi::Pneumatics& descorer);

	/** @brief Applies subsystem startup configuration. */
	void initialize();
	/** @brief Reads driver controls and updates pneumatic outputs. */
	void updateFromController(pros::Controller& controller);
	/** @brief Sets the ball blocker state. */
	void setBallBlocker(bool extended);
	/** @brief Sets the scoring lifter state. */
	void setScoringLifter(bool extended);
	/** @brief Sets the matchloader state. */
	void setMatchloader(bool extended);
	/** @brief Sets the descorer state. */
	void setDescorer(bool extended);

private:
	pros::adi::Pneumatics& m_ballBlocker;
	pros::adi::Pneumatics& m_scoringLifter;
	pros::adi::Pneumatics& m_matchloader;
	pros::adi::Pneumatics& m_descorer;
};

}  // namespace daydream
