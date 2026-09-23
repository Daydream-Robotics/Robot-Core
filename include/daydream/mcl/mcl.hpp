#ifndef _MCL_HPP_
#define _MCL_HPP_

//Inclusions
#include "api.h"

//Created namespace mcl
namespace mcl {

    //robot position structure with x, y, theta
    struct RobotPosition {
        double x;
        double y;
        double theta;
    };

    //particle structure for MCL
    struct Particle {
        double x;                   //x position
        double y;                   //y position
        double theta;               //heading angle
        double weight;              //unnormalized weight
        double normalizedWeight;    //normalized weight
        double logWeight;           //log weight for numerical stability
    };

    //covariance matrix for motion noise
    struct Covariance {
        double xx, xy, xTheta;
        double yx, yy, yTheta;
        double thetaX, thetaY, thetaTheta;
    };

    //initializes MCL system with uniformly distributed particles
    void initMcl(const lemlib::Pose& initPose);
    //main MCL update loop running in background task
    void mclUpdate();
    //returns current estimated robot position
    RobotPosition getEstPos();
    //prints particle statistics for debugging
    void printParticleStats();

    //prints all particle data for debugging
    void printParticleData();

    //estimated robot position from MCL
    extern RobotPosition estPos;
    //mutex for thread-safe position updates
    extern pros::Mutex posUpdateMutex;

}
#endif

