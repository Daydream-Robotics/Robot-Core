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

    /** 
     * @brief gets the yaw of the robot
     * @note Counter clockwise is positive
     * @returns returns yaw/heading in degrees bounded by [-180, 180]
     * @retval	-180.1	IMU disconnected
     * @retval	-180.2	IMU calibrating
     * @retval	-180.3	Pros communication failure
    */
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
