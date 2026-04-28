// include/autonomous.hpp


#pragma once

#include "pid.hpp"
#include "odometry.hpp"

// Autonomous class containing
class Autonomous {
    // Allow AutoTuner to access private members (PIDs)
    friend class AutoTuner;

    public:
        // Constructor
        Autonomous();

        // Turn to a target_heading [-180, 180]
        void turnTo(double targetHeading);

        // Travel a specified distance with speed and heading with a timer (s) exit
        double travel(double distance, double speed, double targetHeading, double timer_s = 0.0);

        bool travelToPoint(double targetX, double targetY, double maxSpeed=200, bool reverse=false, int timer=-1);

    private:

        // Distance PID controller
        PID distancePID;

        // Heading PID controller
        PID headingPID;

        // Turning PID controller
        PID turnPID;

        // Acceleration limit for takeoff (in/s^2)
        double accelLimitRate = 200.0; // TODO: lo: 90 hi: 140 

        // Time for takeoff (s)
        double takeoffRampTime = 0.35; // TODO: tune

        // Immediate stop threshold for movement (in)
        double STOPTHRESHOLD = 0.1; // TODO: tune

};