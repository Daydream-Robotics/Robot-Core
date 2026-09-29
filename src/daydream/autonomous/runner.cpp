/**
 * @file runner.cpp
 * @brief Project interface or implementation.
 */

#include "daydream/autonomous/runner.hpp"

namespace daydream {
namespace autonomous {

void run(Drivetrain& drivetrain, Intake& intake, Pneumatics& pneumatics,
         Odometry& odometry) {
	static_cast<void>(drivetrain);
	static_cast<void>(intake);
	static_cast<void>(pneumatics);
	static_cast<void>(odometry);
	/// Add autonomous selection and routine sequencing here.
}

}  // namespace autonomous
}  // namespace daydream
