// src/pid.cpp

#include "daydream/motion/control/pid.hpp"

#include <cmath>

PID::PID(double p, double i, double d, double integralStartThreshold) : kP(p), kI(i), kD(d), integralStartThreshold(integralStartThreshold) {
    reset();
}

double PID::compute(double current, bool usesAngle) {
    using clock = std::chrono::steady_clock;

    // Determine time since last step
    auto currentTime = clock::now();
    std::chrono::duration<double> elapsedDuration = currentTime - lastComputeTime;
    double dt = elapsedDuration.count();
    if (dt <= 0.0) dt = 1e-3;
    lastComputeTime = currentTime;

    error = target - current;
    if (usesAngle) {
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
    }


    // Integral
    if (std::fabs(error) < integralStartThreshold) {
        integral += error * dt;
    } else {
        integral = 0.0;
    }

    // Derivative
    derivative =  prevError ? (error - prevError) / dt : 0;
    prevError = error;

    // Output
    output = kP * error + kI * integral + kD * derivative;
    return output;
}

void PID::setTarget(double target, bool resetPID) { 
    this->target = target; 
    if (resetPID) reset();
}

void PID::setConstants(double p, double i, double d) {
    kP = p;
    kI = i;
    kD = d;
}

void PID::reset() {
    // Reset collected error
    prevError = 0;
    error = 0;
    integral = 0;
    derivative = 0;
    output = 0;

    // Reset counters
    smallCounter = 0;
    bigCounter = 0;
    velocityCounter = 0;

    // Initialize start time
    startTime = std::chrono::steady_clock::now();
    lastComputeTime = startTime;
}

void PID::setExitCondition(
    double smallError, int smallTime,
    double bigError, int bigTime,
    double velocityThreshold, int velocityTime, 
    int timeout
) {
    this->smallError = smallError;
    this->smallTime = smallTime;
    this->bigError = bigError;
    this->bigTime = bigTime;
    this->velocityThreshold = velocityThreshold;
    this->velocityTime = velocityTime;
    this->timeout = timeout;
}

PID::ExitState PID::checkExitCondition(double currentVelocity) {
    using clock = std::chrono::steady_clock;
    auto currentTime = clock::now();

    // Timeout
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        currentTime - startTime
    ).count();

    if (timeout > 0 && elapsed_ms > timeout)
        return TIMEOUT;

    // Small error
    if (std::fabs(error) < smallError) {
        smallCounter += 10;
        if (smallCounter >= smallTime)
            return SMALL_EXIT;
    } else {
        smallCounter = 0;
    }

    // Big error
    if (std::fabs(error) < bigError) {
        bigCounter += 10;
        if (bigCounter >= bigTime)
            return BIG_EXIT;
    } else {
        bigCounter = 0;
    }

    // Velocity
    if (std::fabs(currentVelocity) < velocityThreshold) {
        velocityCounter += 10;
        if (velocityCounter >= velocityTime)
            return VELOCITY_EXIT;
    } else {
        velocityCounter = 0;
    }

    return RUNNING;
}
