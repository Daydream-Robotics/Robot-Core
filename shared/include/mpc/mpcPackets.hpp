#ifndef _MPC_PACKETS_HPP_
#define _MPC_PACKETS_HPP_

#include "mpc/mpcConfig.hpp"
#include <cstdint>

//binary serial packets between the brain (MPCSerial) and the Nano (microcontroller_main)
//both sides include this header so the layouts can't drift apart

//bump whenever either packet's layout or meaning changes, so a brain/Nano built from different versions logs an error
//it is the first byte of every packet, so it stays readable even when the layouts differ
constexpr uint8_t MPC_PROTOCOL_VERSION = 1;

//brain -> Nano: state estimate + reference trajectory
#pragma pack(push, 1)
struct MPCUpdatePacket {
    uint8_t protocol_version; //MPC_PROTOCOL_VERSION
    //1 = new path: Nano resets its controller before solving
    uint8_t reset_controller;
    uint8_t reserved[2]; //keeps the floats below 4-byte aligned
    float pose_x; //[in]
    float pose_y; //[in]
    float pose_theta; //[rad]
    float omega_L; //[rad/s]
    float omega_R; //[rad/s]
    float V_battery; //[V]
    float I_total; //[A] (get from battery)
    //voltage the brain is applying right now, so the Nano's delta-u limit starts from it (not from its own last output)
    float V_left_applied; //[V]
    float V_right_applied; //[V]
    float track_width; //[in] brain's TRACK_WIDTH (config/constants.h), the Nano builds its model from it
    //F+1 reference poses (x, y, theta)
    float z_desired[(mpc_config::F + 1) * 3];
};
#pragma pack(pop)

//Nano -> brain: optimal input voltages
#pragma pack(push, 1)
struct MPCControlPacket {
    uint8_t protocol_version; //MPC_PROTOCOL_VERSION
    uint8_t reserved[3]; //keeps the floats below 4-byte aligned
    float V_left; //[V]
    float V_right; //[V]
};
#pragma pack(pop)

#endif
