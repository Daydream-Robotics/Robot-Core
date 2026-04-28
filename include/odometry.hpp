#pragma once

typedef struct WheelLengths {
    double parallel;
    double perpendicular;
} WheelLengths;

typedef struct Position {
    double x;
    double y;
} Position;

class Odometry {

public:


    // Update position and orientation
    void updatePose(void);

    // Returns the current yaw (deg) from IMU; Returns -1 if IMU failure
    double getYaw(void);
    
    double getPosX();
    double getPosY();

    // Set the robot's current position
    void setPose(double x, double y);
    
    // Background task entry point
    static void odomTask();

    // Returns struct of distances travelled by Odometry Wheels
    WheelLengths getOdomWheelTravel(void);

    double getParallelVel();

    // x-position of bot (inches)
    double pos_x = 0.0;

    // y-position of bot (inches)
    double pos_y = 0.0;

    // Heading of bot (rads)
    double heading = 0.0;
};

extern Odometry odom;
