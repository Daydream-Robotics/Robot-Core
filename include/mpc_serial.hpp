#ifndef _MPC_SERIAL_HPP_
#define _MPC_SERIAL_HPP_

#include "motionController.hpp"
#include "odometry.hpp"
#include "arclengthSplining.hpp"

#include <cstddef>
#include <string>

class MPCSerial : public MotionController {
public:
    struct Params {
            double r; //wheel radius (in)
            double L; //track width (in)
            double a; //motor time-constant
            double b; //motor gain constant
            double h; //sampling period (s)
            double p_x; //output weight for x
            double p_y; //output weight for y
            double p_theta; //output weight for theta
            double q_u; //input penalty
            double q_zero_multiplier; //multiplier for first Q_i
            double q_final_multiplier; //multiplier for last Q_i
            double p_final_multiplier; //multiplier for last P_i
            double V_max; //voltage max
            double R_internal; //internal resistance of battery
            double delta_u_max; //max positive change in voltage between steps
            double delta_u_min; //max negative change in voltage between steps
            double x_field_max; //max allowed x position on field
            double y_field_max; //max allowed y position on field
            double omega_motor_max; //max allowed rpm
            double V_left_applied; //starting Voltage left
            double V_right_applied; //starting Voltage right
            std::size_t V; //control horizon
            std::size_t F; //prediction horizon

            Params(
                double wheelRadius,
                double trackWidth,
                double motorA,
                double motorB,
                double samplePeriod,
                double pX,
                double pY,
                double pTheta,
                double qU,
                double qZeroMultiplier,
                double qFinalMultiplier,
                double pFinalMultiplier,
                double maxVoltage,
                double rBattery,
                double maxDeltaU,
                double minDeltaU,
                double maxFieldX,
                double maxFieldY,
                double maxMotorOmega, 
                double voltageLeft,
                double voltageRight,
                std::size_t controlHorizon,
                std::size_t predictionHorizon
            );
        };

    explicit MPCSerial(const Params& params);
    ~MPCSerial() override = default;

    WheelVelocities compute(const Pose&, const ALS_Path&, std::size_t&) override;

    WheelVelocities compute(const Pose& currentPose, const ALS_Path& path, std::size_t& closestSampleIdx, double omega_L, double omega_R, double V_battery, double I_total);

private:

    Params m_params;
    

};

#endif