//Inclusions
#include "main.h"
#include "autons.hpp"
#include "display.hpp"
#include "control/mcl.hpp"
#include "control/odom.hpp"
#include "control/sensors.hpp"

#include <cstring>
#include <vector>


static std::vector<int> left_motors = drive::left_motors;
static std::vector<int> right_motors = drive::right_motors;

//Initialization Method, for when program turns on
void initialize() {

    drive::init();
    maelstrom::logging::init(true, true, left_motors, right_motors, 42);
    pros::Task error_logger(maelstrom::logging::robot_faults_log);
    //Runs the initialization codes
    intake::init();

    // drive::chassis.drive_brake_set(MOTOR_BRAKE_COAST);
}

//Method for when Robot is Disabled
void disabled() {}

//method for when the robot is in competition-initialized, ie. plugged into field control and disabled
void competition_initialize() {
    //loads the screen for auton selector, skills selector, profile selector, debug info, and profile selector
   initialization_display();
}

//Method for Autonomous period
void autonomous() {
    run_selected_auto();
    auton_complete = true;
    run_selected_GIF();
}


//method for during drive control
void opcontrol() {
    const lemlib::Pose driver_mcl_init_pose = {0, 0, 0}; // change to your actual field pose for driver testing
    mcl::init_mcl(driver_mcl_init_pose);
    drive::left_motor_group.set_brake_mode(MOTOR_BRAKE_COAST);
    drive::right_motor_group.set_brake_mode(MOTOR_BRAKE_COAST);
// //    maelstrom::logging::write_to_file("Good Luck", maelstrom::logging::E_DATA_LOG);
    drive::chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    double theta_heading = NAN;
    /* Put the controls for driving
    the drivetrain in a PROS Task */
    pros::Task drive(drive::control, TASK_PRIORITY_DEFAULT + 4);
    /* Put the controls for intaking
        in a PROS Task */          
    pros::Task intake(intake::control, TASK_PRIORITY_DEFAULT + 3);
    /* Put the controls for controlling the
        pneumatics in a PROS Task */
    pros::Task pneumatics(pneumatics::control, TASK_PRIORITY_DEFAULT + 2);
    
    // maelstrom::logging::set_robot_coords(drive::chassis.odom_x_get(), drive::chassis.odom_y_get(), drive::chassis.odom_theta_get());
    const pros::Task coords_logging(maelstrom::logging::robot_coords_log, TASK_PRIORITY_MIN);
    
    if ((auto_type == AUTONOMOUS_SKILLS)){
		driver_skills();
	}
   maelstrom::logging::task_complete("Auton", true);
    // if (selected_debug_option == 1) {
        // pid_screen_display(flywheel::pid.kP, flywheel::pid.kI, flywheel::pid.kD);
        // const pros::Task pid_update(pid_constant_updating);
    // }
    // run_selected_GIF();
    lemlib::Pose pose = drive::chassis.getPose();
    

    while (true) {

        pros::delay(300);
    }
}
