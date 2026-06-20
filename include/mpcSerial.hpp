#ifndef _MPC_SERIAL_HPP_
#define _MPC_SERIAL_HPP_

#include "motionController.hpp"
#include "odometry.hpp"
#include "arclengthSplining.hpp"
#include "serialProtocol.hpp"

#include <cstddef>
#include <string>

//offboard MPC controller: runs on vex brain then sends state/references to embedded solver on microcontroller via serial, receives optimal voltages back.

class MPCSerial : public MotionController {
public:
    //timing and drivetrain parameters
    struct Params {
        double h; //sample period (s)
        double gear_ratio; //gear multiplier: (driver teeth / driven teeth)

        Params(double frequency, double ratio = 1.0);
    };

    //stores prediction horizon (must match microcontroller)
    static constexpr std::size_t F = 15;

    explicit MPCSerial(const Params& params);
    ~MPCSerial() override = default;

    void reset() override {}
    //public compute: sends parameters plus robot telematry to private compute, gets voltages
    WheelVelocities compute(const Pose& currentPose, const ALS_Path& path, std::size_t& closestSampleIdx, PathFollower::Flags flag) override;

    //motor model identification
    static void identifyMotorModel();

private:
    //binary packet def for the microcontroller the data it needs to compute input volatges
    #pragma pack(push, 1)
    struct MPCUpdatePacket {
        float pose_x; //[in]
        float pose_y; //[in]
        float pose_theta; //[rad]
        float omega_L; //[rad/s]
        float omega_R; //[rad/s]
        float V_battery; //[V]
        float I_total; //[A] (get from battery)
        float z_desired[F*3];
    };
    #pragma pack(pop)
    //binary packet def to send input voltages
    #pragma pack(push, 1)
    struct MPCControlPacket {
        float V_left; //[V]
        float V_right; //[V]
    };
    #pragma pack(pop)

    //interpolated point on path with arc length parameterization
   struct InterpSample {
       double s = 0.0; // arc length along path (in)
       double x = 0.0; // x position (in)
       double y = 0.0; // y position (in)
       double theta = 0.0; // heading (rad)
       double v = 0.0; // path velocity (in/s)
   };

    SerialProtocol serial; //serial link to microcontroller
    Params m_params; //timing and gear ratio

    //private compute: sends full state + reference to microcontroller, gets voltages
    WheelVelocities compute(const Pose& currentPose, const ALS_Path& path, std::size_t& closestSampleIdx, PathFollower::Flags flag, double omega_L, double omega_R, double V_battery, double I_total);
    //find closest point on spline for given arc length
    InterpSample sampleAtArcLength(const std::vector<Sample>& samples, double sQuery);
    //pack current state + build reference trajectory into update packet
    MPCUpdatePacket buildUpdatePacket(const Pose& currentPose, const std::vector<Sample>& samples, std::size_t idx, PathFollower::Flags flag, double omega_L, double omega_R, double V_battery, double I_total);

    //get motor model
    static double estimateA(const std::vector<double>& time, const std::vector<double>& omega, double omega_ss);
    static double estimateB(double a, double omega_ss, double voltage);
    static void runSingleIdentificationTest(int voltage, double& out_a, double& out_b);
};

#endif