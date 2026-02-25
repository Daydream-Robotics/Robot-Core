// include/autonomous.hpp

#pragma once

#include "pid.hpp"

typedef struct WheelLengths {
    double parallel;
    double perpendicular;
} WheelLengths;

typedef struct Position {
    double x;
    double y;
} Position;

// Autonomous class containing
class Autonomous {

    public:
        // Constructor
        Autonomous();

        // Turn to a target_heading [-180, 180]
        void turnTo(double targetHeading);

        // Travel a specified distance with speed and heading with a timer (s) exit
        void travel(double distance, double speed, double targetHeading, double timer_s = 0.0);

        // Update position and orientation
        void updatePose(void);

        double getYaw(void);

    private:

        // Distance PID controller
        PID distancePID;

        // Heading PID controller
        PID headingPID;

        // Turning PID controller
        PID turnPID;

        // x-position of bot (inches)
        double pos_x = 0.0;

        // y-position of bot (inches)
        double pos_y = 0.0;

        // Heading of bot (rads)
        double heading = 0.0;

        // Acceleration limit for takeoff (in/s^2)
        double accelLimitRate = 200.0; // TODO: tune

        // Time for takeoff (s)
        double takeoffRampTime = 0.35; // TODO: tune

        // Immediate stop threshold for movement (in)
        double STOPTHRESHOLD = 0.1; // TODO: tune

        

        // Returns the current yaw (deg) from IMU; Returns -1 if IMU failure
        

        // Returns struct of distances travelled by Odometry Wheels
        WheelLengths getOdomWheelTravel(void);

};