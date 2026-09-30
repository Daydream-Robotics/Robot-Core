// include/pid.hpp

#pragma once

#include <chrono>

// PID class implementation with parameterizable exit states
class PID {
    private:
        // Constants
        double kP, kI, kD;
        double integralStartThreshold;
        
        // State
        double target;
        double error, prevError;
        double integral;
        double derivative;
        double output;
        
        // Exit conditions
        double smallError, bigError;
        int smallTime, bigTime;
        double velocityThreshold;
        int velocityTime, timeout;
        
        int smallCounter, bigCounter, velocityCounter;
        std::chrono::steady_clock::time_point startTime, lastComputeTime ;
        
    public:
        // Initialize PID with constants
        PID(double p, double i, double d, double integralStartThreshold);
        
        // Set target and reset error
        void setTarget(double target, bool resetPID = true);

        // Set PID constants
        void setConstants(double p, double i, double d);

        // Returns next output value based on current value
        double compute(double current, bool usesAngle = false);

        // Set parameters for exiting PID computation
        void setExitCondition(double smallError, int smallTime,
            double bigError, int bigTime, double velocityThreshold,
            int velocityTime, int timeout);
        
        // Enum for different states for exiting PID computation
        enum ExitState { RUNNING, SMALL_EXIT, BIG_EXIT, VELOCITY_EXIT, TIMEOUT };
        
        // Determine PID running state and reason for exiting
        ExitState checkExitCondition(double current_velocity);
        
        // Reset error, counters, and time state
        void reset();
};