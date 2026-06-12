#include "mpc_serial.hpp"

#include <cmath>
#include <algorithm>

MPCSerial::Params::Params(
    double frequency
)
    : h(frequency) {}

MPCSerial::MPCSerial(const Params& params)
    : m_params(params), serial(1024, ',', '|', '\n', SerialProtocol::Mode::BINARY)
{

}

static inline double wrapAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

MPCSerial::InterpSample MPCSerial::sampleAtArcLength(const std::vector<Sample>& samples, double sQuery) {
    InterpSample result;
    if (samples.empty()) {
        return;
    }
    if (sQuery <= samples.front().s) {
        result.x = samples.front().x;
        result.y = samples.front().y;
        result.theta = samples.front().heading;
        result.v = samples.front().v;
        result.s = samples.front().s;
        return result;
    }
    if (sQuery >= samples.back().s) {
        result.x = samples.back().x;
        result.y = samples.back().y;
        result.theta = samples.back().heading;
        result.v = samples.back().v;
        result.s = samples.back().s;
        return result;
    }

    std::size_t left = 0;
    std::size_t right = samples.size()-1;

    while (left+1 < right) {
        std::size_t mid = left + (right-left)/2;
        if (samples[mid].s <= sQuery) {
            left = mid;
        }
        else {
            right = mid;
        }
    }
    const Sample& a = samples[left];
    const Sample& b = samples[right];

    double ds = b.s - a.s;
    double alpha = (std::abs(ds) < 1e-12) ? 0.0 : (sQuery - a.s) / ds;
    double dtheta = wrapAngle(b.heading-a.heading);

    result.x = a.x + alpha * (b.x-a.x);
    result.y = a.y + alpha * (b.y-a.y);
    result.theta = wrapAngle(a.heading + alpha * dtheta);
    result.v = a.v + alpha * (b.v-a.v);

    return result;
}

MPCSerial::MPCUpdatePacket MPCSerial::buildUpdatePacket(const Pose& currentPose, const std::vector<Sample>& samples, std::size_t idx, double omega_L, double omega_R, double V_battery, double I_total) {
    MPCUpdatePacket p{};

    // copy current robot state
    p.pose_x = (float)currentPose.x;
    p.pose_y = (float)currentPose.y;
    p.pose_theta = (float)currentPose.theta;

     // copy telemetry
    p.omega_L = (float)omega_L;
    p.omega_R = (float)omega_R;
    p.V_battery = (float)V_battery;
    p.I_total = (float)I_total;

    // build z_desired
    double s = samples[idx].s;
    for (int i = 0; i < F; i++) {
        InterpSample curr = sampleAtArcLength(samples, s);
        double v = std::max(curr.v, 0.0);
        s += v * m_params.h;
        if (s > samples.back().s) {
            s = samples.back().s;
        }
        InterpSample ref = sampleAtArcLength(samples, s);
        int base = i * 3;
        p.z_desired[base + 0] = (float)ref.x;
        p.z_desired[base + 1] = (float)ref.y;
        p.z_desired[base + 2] = (float)ref.theta;
    }
    return p;
}

WheelVelocities MPCSerial::compute(const Pose& currentPose, const ALS_Path& path, std::size_t& closestIdx, double omega_L, double omega_R, double V_battery, double I_total) {
    MPCUpdatePacket req = buildUpdatePacket(currentPose, path.getSamples(), closestIdx, omega_L, omega_R, V_battery, I_total);
    serial.send(req);
    std::optional<MPCControlPacket> res = serial.receive<MPCControlPacket>();
    if (!res.has_value()) {
        return {0, 0}; 
    }
    return {
        res->V_left,
        res->V_right
    };
}