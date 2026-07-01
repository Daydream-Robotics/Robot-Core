#include "ramsete.hpp"
#include <cmath>
#include "helpers.hpp"
#include "constants.h"
#include "sd_card_logging.hpp"

// Notes:
// Check that all headings are in radians (also add these to comments)

RamseteController::RamseteController(RamseteConfig config) {
    this->m_b = config.b;
    this->m_zeta = config.zeta;
    this->m_trackWidthInches = config.trackWidthInches;
    this->m_lookaheadInches = config.lookaheadInches;
}


RamseteController::RamseteController(double b, double zeta, double trackWidthInches, double lookaheadInches) {
    this->m_b = b;
    this->m_zeta = zeta;
    this->m_trackWidthInches = trackWidthInches;
    this->m_lookaheadInches = lookaheadInches;
}

Sample RamseteController::getLookaheadSample(const ALS_Path& als_path, std::size_t closestIdx) {
    const std::vector<Sample>& samples = als_path.getSamples();
    double targetS = samples[closestIdx].s + m_lookaheadInches;

    std::size_t idx = closestIdx;
    while (idx + 1 < samples.size() && samples[idx].s < targetS) {
        idx++;
    }
    return samples[idx];
}

WheelVelocities RamseteController::compute(const Pose& currentPose, const ALS_Path& als_path, std::size_t& targetIdx, PathFlag flag) {
    // Sample target = als_path.getSamples()[targetIdx];
    Sample target = getLookaheadSample(als_path, targetIdx);

    // set up virtual pose (this is used to make the robot consider it's back as forward when moving in reverse)
    Pose virtualPose = currentPose;
    if (flag == PathFlag::REVERSE) {
        // rotate robot orientation 180 degrees to consider the back as forward
        virtualPose.theta = angleDiffRad(currentPose.theta + M_PI, 0.0);
    }

    // calculate global displacement errors
    double dx = target.x - currentPose.x;
    double dy = target.y - currentPose.y;

    // project global errors to local coordinate frame
    double e_x = (std::cos(virtualPose.theta) * dx) + (std::sin(virtualPose.theta) * dy);
    double e_y = (-std::sin(virtualPose.theta) * dx) + (std::cos(virtualPose.theta) * dy);
    double e_theta = angleDiffRad(target.heading, virtualPose.theta);

    // calculate target angular velocity
    double targetAngularVel = target.v * target.curvature;

    // calculate gain
    // k = 2 * zeta * sqrt(w_d^2 + b * v_d^2)
    double k = 2.0 * m_zeta * std::sqrt(std::pow(targetAngularVel,2) + (m_b * std::pow(target.v,2)) );

    // calculate linear and angular velocity to command
    double commandLinearVel_preClamp = (target.v * std::cos(e_theta)) + (k * e_x);
    double commmandAngularVel = targetAngularVel + (k * e_theta) + (m_b * target.v * sinc(e_theta) * e_y);

    double maxLinearSpeedInchesPerSecond = (450.0 / 60.0) * (M_PI * DRIVE_WHEEL_DIAMETER_INCHES);
    double commandLinearVel = std::clamp(commandLinearVel_preClamp, -maxLinearSpeedInchesPerSecond, maxLinearSpeedInchesPerSecond);

    // reverse linear velocity in case of reverse
    if (flag == PathFlag::REVERSE) {
        commandLinearVel = -commandLinearVel;
    }

    // get target inches per second for each side
    double leftInchesPerSec = commandLinearVel - (commmandAngularVel * m_trackWidthInches / 2.0);
    double rightInchesPerSec = commandLinearVel + (commmandAngularVel * m_trackWidthInches / 2.0);

    // convert linear wheel speeds from inch/ser to motor rpm
    // TODO: Add gear ratio math
    double inchToRpmConversion = 60.0 / (M_PI * DRIVE_WHEEL_DIAMETER_INCHES);

    WheelVelocities speeds;
    speeds.input = ControlMode::INPUT_VELOCITY;
    speeds.left = leftInchesPerSec * inchToRpmConversion;
    speeds.right = rightInchesPerSec * inchToRpmConversion;

    double left_preClamp = speeds.left;
    double right_preClamp = speeds.right;

    // scale to remain within wheel bounds
    double max_req = std::max(std::abs(speeds.left), std::abs(speeds.right));
    bool wasScaled = false;
    if (max_req > 600.0) {
        speeds.left = speeds.left * (600.0 / max_req);
        speeds.right = speeds.right * (600.0 / max_req);
        wasScaled = true;
    }

    // !DEBUG
    static int debugTick = 0;
    if (++debugTick % 5 == 0) {
        char logBuf[256];
        snprintf(logBuf, sizeof(logBuf),
            "[RAMSETE] idx=%zu flag=%d | pose(%.2f,%.2f,%.3f) target(%.2f,%.2f,%.3f v=%.2f k=%.3f s=%.2f) | e=(%.3f,%.3f,%.3f) | k_gain=%.3f | vCmd=%.2f(pre=%.2f) wCmd=%.3f | L=%.1f(pre=%.1f) R=%.1f(pre=%.1f) scaled=%d",
            targetIdx, static_cast<int>(flag),
            currentPose.x, currentPose.y, currentPose.theta,
            target.x, target.y, target.heading, target.v, target.curvature, target.s,
            e_x, e_y, e_theta,
            k,
            commandLinearVel, commandLinearVel_preClamp, commmandAngularVel,
            speeds.left, left_preClamp, speeds.right, right_preClamp, wasScaled ? 1 : 0);
        printf("%s\n", logBuf);
        LOG(logBuf);
    }

    return speeds;
}


double RamseteController::sinc(double x) {
    if (std::abs(x) < 1e-9) return 1; // Mathematical limit of sin(x)/x as x approaches 0
    return std::sin(x) / x;
}
