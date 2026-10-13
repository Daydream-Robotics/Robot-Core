//Inclusions
#include "api.h"
#include "drive.hpp"
#include "intake.hpp"
//Created namespace pneumatics
namespace pneumatics {
    //defined and initialized controller object
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    //defined and initialized pneumatic objects for solenoids
    pros::adi::Pneumatics low_goal_cylinder('i', false);
    pros::adi::Pneumatics angel_wing('a', false);
    pros::adi::Pneumatics matchload_cylinder('e', false);


    //defined init function for later
    void init() {}

    //defined function to control pneumatic solenoids
    void control() {
        /*While true loop so the code is always running in the 
            background because the function will be run in a task */
        while (true) {
            /* Checks if the down button is pressed if so, it toggles
            the angel wing cylinder*/
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
                angel_wing.toggle();
            }
            /* Checks if the B button is pressed if so, it toggles
            the angel wing cylinder*/
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) { 
                angel_wing.toggle();
            }
            /* Checks if the Y button is pressed if so, it toggles
            the matchloader cylinder*/
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
                matchload_cylinder.toggle();
            }
            //standard 10 ms delay
            pros::delay(10);
        }
    }
}