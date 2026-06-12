#ifndef _MPC_SERIAL_HPP_
#define _MPC_SERIAL_HPP_

#include "motionController.hpp"
#include "odometry.hpp"
#include "arclengthSplining.hpp"
#include "serial_protocol.hpp"

#include <cstddef>
#include <string>

class MPCSerial : public MotionController {
public:

    struct Params {
            double h;

            Params(
                double frequency
            );
        };

    explicit MPCSerial(const Params& params);
    ~MPCSerial() override = default;

    WheelVelocities compute(const Pose&, const ALS_Path&, std::size_t&) override;

    WheelVelocities compute(const Pose& currentPose, const ALS_Path& path, std::size_t& closestSampleIdx, double omega_L, double omega_R, double V_battery, double I_total);
    static constexpr std::size_t F = 40;
private:
    

    #pragma pack(push, 1)
    struct MPCUpdatePacket {
        float pose_x;
        float pose_y;
        float pose_theta;
        float omega_L;
        float omega_R;
        float V_battery;
        float I_total;
        float z_desired[F*3];
    };
    #pragma pack(pop)

    #pragma pack(push, 1)
    struct MPCControlPacket {
        float V_left;
        float V_right;
    };
    #pragma pack(pop)

    struct InterpSample {
        double s = 0.0;
        double x = 0.0;
        double y = 0.0;
        double theta = 0.0;
        double v = 0.0;
    };

    SerialProtocol serial;
    Params m_params;
    InterpSample sampleAtArcLength(const std::vector<Sample>& samples, double sQuery);
    MPCUpdatePacket buildUpdatePacket(const Pose& currentPose, const std::vector<Sample>& samples, std::size_t idx, double omega_L, double omega_R, double V_battery, double I_total);
};

#endif