/**
 * @file intake.cpp
 * @brief Project interface or implementation.
 */

#include "daydream/subsystems/intake.hpp"

namespace daydream {

void Intake::initialize() {}

void Intake::updateFromController(pros::Controller& controller) {
	static_cast<void>(controller);
	/// Add driver input handling here.
}

void Intake::setPower(int millivolts) {
	static_cast<void>(millivolts);
	/// Connect this interface to the intake hardware when its port configuration is finalized.
}

void Intake::stop() {
	setPower(0);
}

}  // namespace daydream
