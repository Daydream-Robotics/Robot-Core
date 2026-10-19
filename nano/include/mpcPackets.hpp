#ifndef _MPC_PACKETS_HPP_
#define _MPC_PACKETS_HPP_

#include "mpcConfig.hpp"
#include <cstdint>

//binary serial packets between the brain (MPCSerial) and the Nano (microcontroller_main)
//both sides include this header so the layouts can't drift apart

//brain -> Nano: state estimate + reference trajectory
#pragma pack(push, 1)
struct MPCUpdatePacket {
    float pose_x; //[in]
    float pose_y; //[in]
    float pose_theta; //[rad]
    float omega_L; //[rad/s]
    float omega_R; //[rad/s]
    float V_battery; //[V]
    float I_total; //[A] (get from battery)
    //F+1 reference poses (x, y, theta)
    float z_desired[(mpc_config::F + 1) * 3];
    //1 = new path: Nano resets its controller before solving
    uint8_t reset_controller;
};
#pragma pack(pop)

//Nano -> brain: optimal input voltages
#pragma pack(push, 1)
struct MPCControlPacket {
    float V_left; //[V]
    float V_right; //[V]
};
#pragma pack(pop)

#endif
