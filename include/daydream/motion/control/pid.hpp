// include/pid.hpp

#pragma once

#include <chrono>

// PID class implementation with parameterizable exit states
class PID {
    private:
        // Constants
        double kP = 0.0, kI = 0.0, kD = 0.0;
        double start_i = 0.0;
        
        // State
        double target = 0.0;
        double error = 0.0, prevError = 0.0;
        double integral = 0.0;
        double derivative = 0.0;
        double output = 0.0;
        
        // Exit conditions (all zero never exits until exit_condition_set() is called)
        double smallError = 0.0, bigError = 0.0;
        int smallTime = 0, bigTime = 0;
        double velocityThreshold = 0.0;
        int velocityTime = 0, timeout = 0;
        
        int smallCounter = 0, bigCounter = 0, velocityCounter = 0;
        std::chrono::steady_clock::time_point startTime, lastTime;
        
    public:
        // Initialize PID with constants
        PID(double p, double i, double d, double start_i);
        // Zero gains, set them with setConstants()
        PID();
        
        // Set target and reset error
        void setTarget(double target, bool resetPID = true);

        // Set PID constants
        void setConstants(double p, double i, double d);

        // Returns next output value based on current value
        double compute(double current, bool usesAngle = false);

        // Set parameters for exiting PID computation
        void exit_condition_set(double smallError, int smallTime,
            double bigError, int bigTime, double velocityThreshold,
            int velocityTime, int timeout);
        
        // Enum for different states for exiting PID computation
        enum ExitState { RUNNING, SMALL_EXIT, BIG_EXIT, VELOCITY_EXIT, TIMEOUT };
        
        // Determine PID running state and reason for exiting
        ExitState exit_condition(double current_velocity);
        
        // Reset error, counters, and time state
        void reset();
};