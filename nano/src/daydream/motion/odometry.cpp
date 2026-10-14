#include "odometry.hpp"

#include <cmath>

namespace {
    constexpr double PI = 3.14159265358979323846;

    // Convert wheel rotation distance to linear distance.
    double wheelRevolutionsToDistance(double revolutions,
                                      double wheelDiameter) {
        return revolutions * PI * wheelDiameter;
    }

    // Wrap an angle to [-PI, PI].
    double normalizeAngle(double angle) {
        while (angle > PI) {
            angle -= 2.0 * PI;
        }

        while (angle < -PI) {
            angle += 2.0 * PI;
        }

        return angle;
    }

    // Calculate the average of two values.
    double average(double a, double b) {
        return (a + b) / 2.0;
    }
}

// Global odometry instance.
// Configure this in your main application as needed.
Odometry odom(OdomConfig{});

// Construct the odometry system.
Odometry::Odometry(const OdomConfig& config)
    : m_config(config) {
}

// Get the current pose.
Pose Odometry::getPose() const {
    return m_currentPosition;
}

// Set the robot's field pose.
void Odometry::setPose(const Pose& pose) {
    m_currentPosition = pose;

    // Preserve the relationship between field heading and IMU heading.
    m_headingOffset = normalizeAngle(pose.theta - m_lastRawTheta);
    m_currentPosition.theta = normalizeAngle(pose.theta);

    // The next update establishes a new baseline.
    m_initialized = false;
}

// Get the current global position.
Position Odometry::getPosition() const {
    return {m_currentPosition.x, m_currentPosition.y};
}

// Get the X coordinate.
double Odometry::getPosX() const {
    return m_currentPosition.x;
}

// Get the Y coordinate.
double Odometry::getPosY() const {
    return m_currentPosition.y;
}

// Get the latest yaw and its error status.
YawResult Odometry::getYaw() const {
    return {
        normalizeAngle(m_sensorData.yaw + m_headingOffset),
        m_sensorData.imuError
    };
}

// Get parallel wheel velocity.
VelocityResult Odometry::getParallelVel() const {
    return {
        m_sensorData.parallelVelocity,
        m_sensorData.trackingWheelError
    };
}

// Get cumulative drive encoder travel.
WheelLengths Odometry::getDriveEncoderTravel() const {
    return {
        m_sensorData.leftDriveDistance,
        m_sensorData.rightDriveDistance
    };
}

// Get cumulative tracking wheel travel.
WheelTravelResult Odometry::getOdomWheelTravel() const {
    return {
        {
            m_sensorData.parallelDistance,
            m_sensorData.perpendicularDistance
        },
        m_sensorData.trackingWheelError
    };
}

// Supply new sensor readings and update odometry.
void Odometry::update(const OdomSensorData& sensorData) {
    m_sensorData = sensorData;
    updatePose();
}

// Reset the sensor history without changing the current field pose.
void Odometry::resetTracking(const OdomSensorData& sensorData) {
    m_sensorData = sensorData;

    m_lastRawTheta = sensorData.yaw;

    m_prevTheta = normalizeAngle(
        sensorData.yaw + m_headingOffset
    );

    m_prevParallel = sensorData.parallelDistance;
    m_prevPerpendicular = sensorData.perpendicularDistance;

    m_prevLeft = sensorData.leftDriveDistance;
    m_prevRight = sensorData.rightDriveDistance;

    m_initialized = true;
}

// Update the global pose from sensor readings.
void Odometry::updatePose() {
    // Do not update position if the IMU is unavailable.
    if (m_sensorData.imuError != OdomError::NONE) {
        return;
    }

    const double rawTheta = m_sensorData.yaw;

    const double theta = normalizeAngle(
        rawTheta + m_headingOffset
    );

    m_lastRawTheta = rawTheta;

    // Use the first valid sensor reading to establish a baseline.
    if (!m_initialized) {
        m_prevTheta = theta;

        m_prevParallel = m_sensorData.parallelDistance;
        m_prevPerpendicular = m_sensorData.perpendicularDistance;

        m_prevLeft = m_sensorData.leftDriveDistance;
        m_prevRight = m_sensorData.rightDriveDistance;

        m_currentPosition.theta = theta;

        m_initialized = true;
        return;
    }

    // Calculate the change in heading.
    const double deltaTheta = normalizeAngle(theta - m_prevTheta);

    double deltaParallel = 0.0;
    double deltaPerpendicular = 0.0;

    if (m_config.useMotorEncoders) {
        // Average left and right drive encoder travel.
        const double deltaLeft =
            m_sensorData.leftDriveDistance - m_prevLeft;

        const double deltaRight =
            m_sensorData.rightDriveDistance - m_prevRight;

        deltaParallel = average(deltaLeft, deltaRight);

        // Drive encoders alone cannot measure sideways movement.
        deltaPerpendicular = 0.0;
    } else {
        // Tracking wheel displacement.
        deltaParallel =
            m_sensorData.parallelDistance - m_prevParallel;

        deltaPerpendicular =
            m_sensorData.perpendicularDistance -
            m_prevPerpendicular;
    }

    // Compensate for tracking wheel offsets during rotation.
    const double localForward =
        deltaParallel -
        (m_config.parallelTrackingWheelOffset * deltaTheta);

    const double localSideways =
        deltaPerpendicular -
        (m_config.perpendicularTrackingWheelOffset * deltaTheta);

    // Convert local movement into global movement.
    double deltaX;
    double deltaY;

    if (std::fabs(deltaTheta) < 1e-9) {
        deltaX =
            localForward * std::cos(theta) -
            localSideways * std::sin(theta);

        deltaY =
            localForward * std::sin(theta) +
            localSideways * std::cos(theta);
    } else {
        // Exact integration over the change in heading.
        const double sinTheta = std::sin(theta);
        const double cosTheta = std::cos(theta);

        const double sinPrev = std::sin(m_prevTheta);
        const double cosPrev = std::cos(m_prevTheta);

        const double avgSin = (sinTheta - sinPrev) / deltaTheta;
        const double avgCos = (cosPrev - cosTheta) / deltaTheta;

        deltaX =
            localForward * avgCos -
            localSideways * avgSin;

        deltaY =
            localForward * avgSin +
            localSideways * avgCos;
    }

    // Update global position.
    m_currentPosition.x += deltaX;
    m_currentPosition.y += deltaY;
    m_currentPosition.theta = theta;

    // Save readings for the next update.
    m_prevTheta = theta;

    m_prevParallel = m_sensorData.parallelDistance;
    m_prevPerpendicular = m_sensorData.perpendicularDistance;

    m_prevLeft = m_sensorData.leftDriveDistance;
    m_prevRight = m_sensorData.rightDriveDistance;
}