#ifndef _MCL_HPP_
#define _MCL_HPP_

//Inclusions
#include "api.h"

//Created namespace mcl
namespace mcl {

    //robot position structure with x, y, theta
    struct robot_position {
        double x;
        double y;
        double theta;
    };

    //particle structure for MCL
    struct Particle {
        double x;           //x position
        double y;           //y position
        double theta;       //heading angle
        double w;           //unnormalized weight
        double W;           //normalized weight
        double log_w;       //log weight for numerical stability
    };

    //covariance matrix for motion noise
    struct Covariance {
        double x_x, x_y, x_theta;
        double y_x, y_y, y_theta;
        double theta_x, theta_y, theta_theta;
    };

    //initializes MCL system with uniformly distributed particles
    void init_mcl(const lemlib::Pose& init_pose);
    //main MCL update loop running in background task
    void mcl_update();
    //returns current estimated robot position
    robot_position get_est_pos();
    //prints particle statistics for debugging
    void print_particle_stats();

    //prints all particle data for debugging
    void print_particle_data();

    //estimated robot position from MCL
    extern robot_position est_pos;
    //mutex for thread-safe position updates
    extern pros::Mutex pos_update_mutex;

}
#endif

