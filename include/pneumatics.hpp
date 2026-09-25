//header guard
#ifndef _PNEUMATICS_HPP_
#define _PNEUMATICS_HPP_

//inclusions
#include "api.h"

namespace pneumatics {
    //declarations of pneumatics objects
    extern pros::adi::Pneumatics low_goal_cylinder;
    extern pros::adi::Pneumatics angel_wing;
    extern pros::adi::Pneumatics matchload_cylinder;

    //declarations of namespace functions
    void init();
    void control();
}  
//end of if statement
#endif