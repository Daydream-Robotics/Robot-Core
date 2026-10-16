#pragma once

#include <mutex>

/**
 * @brief Physical configuration for the odometry system.
 *
 * All distances are in inches.
 */
struct OdomConfig {
    double parallelWheelDiameter = 0.0;
    double perpendicularWheelDiameter = 0.0;

    double parallelTrackingWheelOffset = 0.0;
    double perpendicularTrackingWheelOffset = 0.0;

    bool useMotorEncoders = false;
    double driveWheelDiameter = 0.0;
};

/**
 * @brief Errors that may occur in the odometry system.
 */
enum class OdomError {
    NONE,
    IMU_DISCONNECTED,
    IMU_CALIBRATING,
    IMU_COMMUNICATION_ERROR,
    TRACKING_WHEEL_ERROR
};

/**
 * @brief IMU yaw measurement and associated status.
 *
 * Yaw is expressed in radians.
 */
struct YawResult {
    double yaw = 0.0;
    OdomError error = OdomError::NONE;
};

/**
 * @brief Distance traveled by the tracking wheels, in inches.
 */
struct WheelLengths {
    double parallel = 0.0;
    double perpendicular = 0.0;
};

/**
 * @brief Tracking wheel travel and associated status.
 */
struct WheelTravelResult {
    WheelLengths travel{};
    OdomError error = OdomError::NONE;
};

/**
 * @brief Velocity measurement in inches per second.
 */
struct VelocityResult {
    double velocity = 0.0;
    OdomError error = OdomError::NONE;
};

/**
 * @brief Global position in inches.
 */
struct Position {
    double x = 0.0;
    double y = 0.0;
};

/**
 * @brief Robot pose in the field.
 *
 * Coordinate convention:
 * +X = forward
 * +Y = left
 * +theta = counterclockwise
 *
 * Theta is expressed in radians.
 */
struct Pose {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

/**
 * @brief Sensor measurements supplied by the computer-side
 * communication layer.
 *
 * Tracking wheel distances must be cumulative distances in inches.
 * IMU yaw must be in radians.
 * Motor encoder distances must be cumulative distances in inches.
 */
struct OdomSensorData {
    double yaw = 0.0;

    double parallelDistance = 0.0;
    double perpendicularDistance = 0.0;

    double leftDriveDistance = 0.0;
    double rightDriveDistance = 0.0;

    double parallelVelocity = 0.0;

    OdomError imuError = OdomError::NONE;
    OdomError trackingWheelError = OdomError::NONE;
};

/**
 * @brief Tracks the robot's global position and heading.
 *
 * This class contains no PROS or VEX hardware dependencies.
 * Sensor readings must be provided by the application.
 */
class Odometry {
public:
    /**
     * @brief Constructs an odometry instance.
     */
    explicit Odometry(const OdomConfig& config);

    /**
     * @brief Gets the latest robot pose.
     */
    Pose getPose() const;

    /**
     * @brief Sets the robot's field pose.
     */
    void setPose(const Pose& pose);

    /**
     * @brief Gets the robot's global position.
     */
    Position getPosition() const;

    /**
     * @brief Gets the robot's X coordinate in inches.
     */
    double getPosX() const;

    /**
     * @brief Gets the robot's Y coordinate in inches.
     */
    double getPosY() const;

    /**
     * @brief Gets the latest yaw and IMU status.
     *
     * Yaw is returned in radians.
     */
    YawResult getYaw() const;

    /**
     * @brief Gets the parallel wheel or drive velocity.
     *
     * Velocity is in inches per second.
     */
    VelocityResult getParallelVel() const;

    /**
     * @brief Gets cumulative left and right drive encoder travel.
     */
    WheelLengths getDriveEncoderTravel() const;

    /**
     * @brief Gets cumulative tracking wheel travel.
     */
    WheelTravelResult getOdomWheelTravel() const;

    /**
     * @brief Supplies new sensor measurements and updates the pose.
     *
     * Call this whenever fresh sensor data arrives from the V5 Brain.
     */
    void update(const OdomSensorData& sensorData);

    /**
     * @brief Resets the odometry history using the current sensor data.
     *
     * Call this after changing the field pose or resetting encoders.
     */
    void resetTracking(const OdomSensorData& sensorData);

private:
    OdomConfig m_config{};

    Pose m_currentPosition{};

    // Protects pose and sensor state if multiple threads access the class.
    mutable std::mutex m_mutex;

    // Latest sensor measurements.
    OdomSensorData m_sensorData{};

    // Previous sensor readings used for incremental odometry.
    double m_prevTheta = 0.0;
    double m_prevParallel = 0.0;
    double m_prevPerpendicular = 0.0;
    double m_prevLeft = 0.0;
    double m_prevRight = 0.0;

    // Difference between field heading and raw IMU heading.
    double m_headingOffset = 0.0;

    double m_lastRawTheta = 0.0;

    bool m_initialized = false;

    /**
     * @brief Updates the pose using the latest sensor data.
     * Caller must hold m_mutex.
     */
    void updatePose();
};

extern Odometry odom;