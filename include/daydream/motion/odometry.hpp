#pragma once

#include "main.h"

struct OdomConfig {
    double parallelWheelDiameter;
    double perpendicularWheelDiameter;
    double parallelTrackingWheelOffset;
    double perpendicularTrackingWheelOffset;
    bool useMotorEncoders = false;
    double driveWheelDiameter = 0.0;
};

/**
 * @enum OdomError
 * @brief Represents errors that may be encountered by odometry system.
 */
enum class OdomError {
    NONE,
    IMU_DISCONNECTED,
    IMU_CALIBRATING,
    IMU_COMMUNICATION_ERROR,
    TRACKING_WHEEL_ERROR
};

/**
 * @struct YawResult
 * @brief Stores yaw measurement and associated odometry error
 */
struct YawResult {
    double yaw{0.0}; /**< The yaw measurement in radians */
    OdomError error{OdomError::NONE}; /**< The IMU error status */
};

struct WheelLengths {
    double parallel; /**< The distance traveled by the parallel tracking wheel in inches */
    double perpendicular; /**< The distance traveled by the perpendicular tracking wheel in inches */
};

/**
 * @struct WheelTravelResult
 * @brief Stores tracking wheel travel mesurements and associated odometry error
 */
struct WheelTravelResult {
    WheelLengths travel{0.0, 0.0}; /**< The distances traveled by the parallel and perpendicular tracking wheels in inches */
    OdomError error{OdomError::NONE}; /**< The tracking wheel error status */
};

/**
 * @struct VelocityResult
 * @brief Stores velocity measurement and associated odometry error
 */
struct VelocityResult {
    double velocity{0.0}; /**< The velocity measurement in inches per second */
    OdomError error{OdomError::NONE}; /**< The velocity error status */
};

/**
 * @struct Position
 * @brief Stores the global position of the robot
 */
struct Position {
    double x; /**< The x-position of the robot in inches */
    double y; /**< The y-position of the robot in inches */
};

/**
 * @brief stores a pose of the robot
 * @note Made for the frame: +X forward, +Y left, CCW positive
 */
struct Pose {
    double x; /**< The x-position of the robot in inches */
    double y; /**< The y-position of the robot in inches */
    double theta; /**< The heading of the robot in radians */
};

/**
 * @class Odometry
 * @brief Tracks robots global position and heading on the field
 */
class Odometry {
    
public:

    /**
     * @brief Constructs Odometry tracking instance
     * @param config The physical odometry dimensions of the robot
     */
    Odometry(OdomConfig config);

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
     * @brief Gets the robot's global position
     * @returns  position  A position struct of the the robot's latest global position
     */
    Position getPosition();

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
     * @returns returns yaw/heading in degrees and error status
     */
    YawResult getYaw(void);
    
    /**
     * @brief gets the current velocity of the parallel tracking wheel or drive motors
     * @returns velocity in inches per second
     */
    VelocityResult getParallelVel();
    
    /**
     * @brief Returns struct of distances travelled by drive encoders when using internal encoders
     */
    WheelLengths getDriveEncoderTravel(void);

    /**
     * @brief Background task entry point
     */
    static void odomTask();

private:
    // the config for the robot
    OdomConfig m_config;

    // stores the latest global position and heading on the robot
    Pose m_currentPosition = {0, 0, 0};

    // ROTS mutex to keep data task safe
    pros::Mutex m_mutex;

    /**
     * @brief Calculates and updates the robot's global pose
     * @note This function is safe to be called continuously in a background task
     * @warning In the case of an IMU failure this function stops updating the latest position
     */
    void updatePose(void);

    /**
     * @brief Returns struct of distances travelled by Odometry Wheels
     */
    WheelTravelResult getOdomWheelTravel(void);
  
    // previous state tracking
    double m_prevTheta = 0;
    double m_prevParallel = 0;
    double m_prevPerpendicular = 0;
    double m_prevLeft = 0;
    double m_prevRight = 0;
    double m_headingOffset = 0.0; //field heading = raw IMU heading + offset
    double m_lastRawTheta = 0.0; //latest raw IMU heading, used by setPose()
    bool m_initialized = false;
    
};

extern Odometry odom;