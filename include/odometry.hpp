#pragma once
#include "pros/rtos.hpp"

struct WheelLengths {
    double parallel;
    double perpendicular;
};

struct Position {
    double x;
    double y;
};

/**
 * @brief stores a pose of the robot
 * @note Made for the frame: +X forward, +Y left, CCW positive
 */
struct Pose {
    // x-position of bot (inches)
    double x;

    // y-position of bot (inches)
    double y;

    // Heading of bot (rads)
    double theta;
};

/**
 * @class Odometry
 * @brief Tracks robots global position and heading on the field
 */
class Odometry {
    
public:

    /**
     * @brief Calculates and updates the robot's global pose
     * @note This function is safe to be called continuously in a background task
     * @warning In the case of an IMU failure this function stops updating the latest position
     */
    void updatePose(void);

    /**
     * @brief Gets the robot's latest pose
     * @returns The latest Pose of the robot
     */
    Pose getPose();

    /**
     * @brief Sets the robot's current position
     * @param pose The starting pose of the robot
     */
    void setPose(Pose pose);

    /**
     * @brief Gets the robots x position
     * @note task safe
     * @returns The global X coordinate of the robot in inches
     */
    double getPosX();

    /**
     * @brief Gets the robots y position
     * @note task safe
     * @returns The global Y coordinate of the robot in inches
     */
    double getPosY();

    /** 
     * @brief gets the yaw of the robot
     * @note Counter clockwise is positive
     * @returns returns yaw/heading in degrees bounded by [-180, 180]
     * @retval	-180.1	IMU disconnected
     * @retval	-180.2	IMU calibrating
     * @retval	-180.3	Pros communication failure
     */
    double getYaw(void);
    
    /**
     * @brief Returns struct of distances travelled by Odometry Wheels
     */
    WheelLengths getOdomWheelTravel(void);
    
    /**
     * @brief gets the current velocity of the parallel tracking wheel
     * @returns velocity of parallel tracking wheel in inches per second
     */
    double getParallelVel();
    
    /**
     * @brief Background task entry point
     */
    static void odomTask();

private:
    // stores the latest global position and heading on the robot
    Pose m_currentPosition = {0, 0, 0};

    // ROTS mutex to keep data task safe
    pros::Mutex m_mutex;
  
    // previous state tracking
    double m_prevTheta = 0;
    double m_prevParallel = 0;
    double m_prevPerpendicular = 0;
    bool m_initialized = false;
};

extern Odometry odom;
