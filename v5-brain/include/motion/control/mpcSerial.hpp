#ifndef _MPC_SERIAL_HPP_
#define _MPC_SERIAL_HPP_

#include "motion/control/motionController.hpp"
#include "localization/odometry.hpp"
#include "motion/pathing/arclengthSplining.hpp"
#include "protocol/serialProtocol.hpp"
//shared with the Nano (shared/include, via EXTRA_INCDIR in the Makefile)
#include "mpc/mpcConfig.hpp"
#include "mpc/mpcPackets.hpp"
#include "mpc/referenceRollout.hpp"

#include <cstddef>
#include <string>

//offboard MPC controller: runs on vex brain then sends state/references to embedded solver on microcontroller via serial, receives optimal voltages back.

class MPCSerial : public MotionController {
public:
    //gear ratio + reference-gen parameters (timing/drivetrain/motor defaults come from mpcConfig.hpp, shared with the Nano)
    struct Params {
        double gear_ratio; //gear multiplier: (driver teeth / driven teeth)
        mpc_ref::Params ref; //sample period, wheel radius, track width, motor a/b, a_max, v_ref_min

        //trackWidth is also sent to the Nano in every update packet
        Params(double ratio, double trackWidth);
    };

    //prediction horizon (shared with the Nano, sets the packet size)
    static constexpr std::size_t F = mpc_config::F;

    explicit MPCSerial(const Params& params);
    ~MPCSerial() override = default;

    //new path: restart the reference speed and tell the Nano to reset its controller
    void reset() override {
        m_v_ref = -1.0;
        m_reset_pending = true;
    }

    //public compute: sends parameters plus robot telematry to private compute, gets voltages
    WheelVelocities compute(const Pose& currentPose, const ALS_Path& path, std::size_t& closestSampleIdx, PathFlag flag) override;

    //motor model identification
    static void identifyMotorModel(double gear_ratio);

private:
    SerialProtocol serial; //serial link to microcontroller
    Params m_params; //timing and gear ratio
    double m_v_ref = -1.0;
    //set by reset(), sent in every update packet until the Nano answers one (so a dropped packet can't lose it)
    //starts true so a Nano still running from a previous program run resets too
    bool m_reset_pending = true;
    //private compute: sends full state + reference to microcontroller, gets voltages
    WheelVelocities compute(const Pose& currentPose, const ALS_Path& path, std::size_t& closestSampleIdx, PathFlag flag, double omega_L, double omega_R, double V_battery, double I_total);
    //pack current state + build reference trajectory into update packet
    MPCUpdatePacket buildUpdatePacket(const Pose& currentPose, const ALS_Path& path, std::size_t idx, PathFlag flag, double omega_L, double omega_R, double V_battery, double I_total);

    //get motor model
    static double estimateA(const std::vector<double>& time, const std::vector<double>& omega, double omega_ss);
    static double estimateB(double a, double omega_ss, double voltage);
    static void runSingleIdentificationTest(int voltage, double& out_a, double& out_b, double gear_ratio);
};

#endif