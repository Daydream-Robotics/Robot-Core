#ifndef _MPC_CONFIG_HPP_
#define _MPC_CONFIG_HPP_

#include <cstddef>

//single source for robot + MPC constants shared by the brain (MPCSerial), the Nano (microcontroller_main) and DreamSim
//header-only, no Eigen/PROS, so the brain's PROS build can include it too
namespace mpc_config {
    //horizons (compile-time, the serial packet layout depends on F)
    constexpr std::size_t F = 15; //prediction horizon
    constexpr std::size_t V = 15; //control horizon

    constexpr double H = 0.02; //sample period (s)

    //drivetrain
    constexpr double WHEEL_RADIUS = 1.625; //wheel radius (in)
    constexpr double TRACK_WIDTH = 10.5; //track width (in)

    //motor model, from MPCSerial::identifyMotorModel()
    constexpr double MOTOR_A = 16.7211; //motor time-constant
    constexpr double MOTOR_B = 71.3105; //motor gain constant

    //reference-gen limits
    constexpr double A_MAX = 60.0; //max accel of bot (in/s^2)
    constexpr double V_REF_MIN = 3.0; //speed floor (in/s)
}

#endif
