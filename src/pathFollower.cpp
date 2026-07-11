#include "pathFollower.hpp"
#include "subsystems.hpp"
#include "constants.h"
#include "fieldLogger.hpp"

double omega_L = 0.0;
double omega_R = 0.0;
WheelVelocities wheelVelocities;

PathFollower::PathFollower(MotionController& controller)
     : m_controller(controller) {}

void PathFollower::setPath(ALS_Path& path, PathFlag flag, bool logging, const char* baseName) {
    this->logging = logging;
    if (logging) {
        path_log.emplace(LoggerType::PATH, baseName);
    }
    this->flag = flag;
    m_path = &path;
    m_currentSampleIdx = 0;
    m_isFinished = false;
    if (m_path->isValid()) {
        m_distanceFromEnd = m_path->getTotalLength();
    }
    m_controller.reset();
    leftMotors.set_brake_mode(MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode(MOTOR_BRAKE_COAST);
}

bool PathFollower::step() {
    if (m_isFinished || !m_path || !m_path->isValid() || m_path->getSamples().empty()) {
        leftMotors.move_velocity(0);
        rightMotors.move_velocity(0);
        path_log->flush();
        path_log->close();
        return true;
    }

    

    m_currentSampleIdx = m_path->findClosestSampleIndex({currentPose.x, currentPose.y}, m_currentSampleIdx);

    if (m_currentSampleIdx >= m_path->getSamples().size()) {
        printf("[PF-ERROR] m_currentSampleIdx (%zu) is out of bounds (size %zu)!\n", m_currentSampleIdx, m_path->getSamples().size());
        m_isFinished = true;
        return true;
    }

    Sample targetSample = m_path->getSamples()[m_currentSampleIdx];
    m_distanceFromEnd = m_path->getTotalLength() - targetSample.s;

    if (m_distanceFromEnd < END_TOLERANCE) {
        leftMotors.move_velocity(0);
        rightMotors.move_velocity(0);
        path_log->flush();
        path_log->close();
        m_isFinished = true;
        return true;
    }
    // pros::lcd::print(0, "run");
    wheelVelocities = m_controller.compute(currentPose, *m_path, m_currentSampleIdx, flag);
    
    switch(wheelVelocities.input){
        case ControlMode::INPUT_VELOCITY:
            leftMotors.move_velocity(wheelVelocities.left);
            rightMotors.move_velocity(wheelVelocities.right);
            break;
        case ControlMode::INPUT_VOLTAGE:
            leftMotors.move_voltage(wheelVelocities.left);
            rightMotors.move_voltage(wheelVelocities.right);
            break;
        default:
            break;
    }
    // motor state (persist across ticks)

    // convert voltage command (mV) to volts
    double V_L = wheelVelocities.left  / 1000.0;
    double V_R = wheelVelocities.right / 1000.0;

    // first-order motor dynamics — same 'a' and 'b' as MPC Params
    omega_L += (-30.0308 * omega_L + 138.9554 * V_L) * 0.02;
    omega_R += (-30.0308 * omega_R + 138.9554 * V_R) * 0.02;

    // wheel linear velocity — omega is rad/s of the wheel
    double wheelRadius = DRIVE_WHEEL_DIAMETER_INCHES / 2.0;
    double leftIps  = omega_L * wheelRadius;
    double rightIps = omega_R * wheelRadius;

    double forward = (leftIps + rightIps) / 2.0;
    double omega   = (rightIps - leftIps) / 10.5;

    currentPose.x     += forward * std::cos(currentPose.theta) * 0.02;
    currentPose.y     += forward * std::sin(currentPose.theta) * 0.02;
    currentPose.theta += omega * 0.02;
    // pros::lcd::print(0, "x %f", currentPose.x);
    // pros::lcd::print(1, "y %f", currentPose.y);
    if (logging && path_log) {
        path_log->log(Waypoint{targetSample.x, targetSample.y, targetSample.v}, Waypoint{currentPose.x, currentPose.y, 0.0}, pros::millis()/1000.0);
    }
    

    return false;
}