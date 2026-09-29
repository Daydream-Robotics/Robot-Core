/**
 * @file intake.hpp
 * @brief Project interface or implementation.
 */

#pragma once

#include "pros/misc.hpp"

namespace daydream {

/** @brief Intake subsystem interface shared by robot operating modes. */
class Intake {
public:
	/** @brief Applies subsystem startup configuration. */
	void initialize();
	/** @brief Reads driver controls and updates the intake command. */
	void updateFromController(pros::Controller& controller);
	/** @brief Sets intake motor output in millivolts. */
	void setPower(int millivolts);
	/** @brief Stops the intake. */
	void stop();
};

}  // namespace daydream
