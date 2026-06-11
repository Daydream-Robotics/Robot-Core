#include "mpc.hpp"

#include <cmath>
#include <algorithm>

MPCSerial::Params::Params(
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
)
    : r(wheelRadius),
    L(trackWidth),
    a(motorA),
    b(motorB),
    h(samplePeriod),
    p_x(pX),
    p_y(pY),
    p_theta(pTheta),
    q_u(qU),
    q_zero_multiplier(qZeroMultiplier),
    q_final_multiplier(qFinalMultiplier),
    p_final_multiplier(pFinalMultiplier),
    V_max(maxVoltage),
    R_internal(rBattery),
    delta_u_max(maxDeltaU),
    delta_u_min(minDeltaU),
    x_field_max(maxFieldX),
    y_field_max(maxFieldY),
    omega_motor_max(maxMotorOmega),
    V_left_applied(voltageLeft),
    V_right_applied(voltageRight)
    V(controlHorizon)
    F(predictionHorizon) {}

template<std::size_t V, std::size_t F>
MPCController<V, F>::MPCController(const Params& params)
    : m_params(params)
{

}