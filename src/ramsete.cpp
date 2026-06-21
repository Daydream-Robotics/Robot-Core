#include "ramsete.hpp"
#include <cmath>
#include "helpers.hpp"
#include "constants.h"

// Notes:
// Check that all headings are in radians (also add these to comments)

RamseteController::RamseteController(RamseteConfig config) {
    this->m_b = config.b;
    this->m_zeta = config.zeta;
    this->m_trackWidthInches = config.trackWidthInches;
}


RamseteController::RamseteController(double b, double zeta, double trackWidthInches) {
    this->m_b = b;
    this->m_zeta = zeta;
    this->m_trackWidthInches = trackWidthInches;
}


WheelVelocities RamseteController::compute(const Pose& currentPose, const ALS_Path& als_path, std::size_t& targetIdx, PathFlag flag) {
    Sample target = als_path.getSamples()[targetIdx];

    double currentTheta = currentPose.theta;

    if (flag == PathFlag::REVERSE) {
        currentTheta = angleDiffRad(currentTheta + M_PI, 0.0);
    }

    // get global error
    double dx = target.x - currentPose.x;
    double dy = target.y - currentPose.y;

    // get local errors
    double e_x = (std::cos(currentTheta) * dx) + (std::sin(currentTheta) * dy);
    double e_y = (-std::sin(currentTheta) * dx) + (std::cos(currentTheta) * dy);
    double e_theta = angleDiffRad(target.heading, currentTheta);

    // calculate target angular velocity
    double targetAngularVel = target.v * target.curvature;

    // get gain
    double k = 2.0 * m_zeta * std::sqrt(std::pow(targetAngularVel,2) + (m_b * std::pow(target.v,2)) );

    // calculate linear and angular velocity to command
    double commandLinearVel = (target.v * std::cos(e_theta)) + (k * e_x);
    double commmandAngularVel = targetAngularVel + (k * e_theta) + (m_b * target.v * sinc(e_theta) * e_y);

    // reverse linear velocity in case of reverse
    if (flag == PathFlag::REVERSE) {
        commandLinearVel = -commandLinearVel;
    }

    // get target inches per second for each side
    double leftInchesPerSec = commandLinearVel - (commmandAngularVel * m_trackWidthInches / 2.0);
    double rightInchesPerSec = commandLinearVel + (commmandAngularVel * m_trackWidthInches / 2.0);

    // TODO: Add gear ratio math
    double inchToRpmConversion = 60.0 / (M_PI * DRIVE_WHEEL_DIAMETER_INCHES);

    WheelVelocities speeds;
    speeds.input = ControlMode::INPUT_VELOCITY;
    speeds.left = leftInchesPerSec * inchToRpmConversion;
    speeds.right = rightInchesPerSec * inchToRpmConversion;

    return speeds;
}


double RamseteController::sinc(double x) {
    if (std::abs(x) < 1e-9) return 1;
    return std::sin(x) / x;
}
