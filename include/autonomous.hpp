// include/autonomous.hpp

#pragma once

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
        void turn(double target_heading);

        // Travel a specified distance with speed and heading with a timer (s) exit
        void travel(double distance, double speed, double target_heading, double timer_s = 0.0);


    private:
        // x-position of bot (inches)
        double pos_x = 0.0;

        // y-position of bot (inches)
        double pos_y = 0.0;

        // Heading of bot (rads)
        double heading = 0.0;

        // Update position and orientation
        void updatePose(void);

        // Returns the current yaw (deg) from IMU; Returns -1 if IMU failure
        double getYaw(void);

        // Returns struct of distances travelled by Odometry Wheels
        WheelLengths getOdomWheelTravel(void);

};