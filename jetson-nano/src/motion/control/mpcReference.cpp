#include "motion/control/mpcReference.hpp"
#include "utils/mathHelpers.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

template<class Drive, std::size_t F>
MPCReference<Drive, F>::MPCReference(const Params& params)
    : m_params(params),
      m_G(Drive::G(params.geom)),
      m_K(Drive::K(params.geom)) {
    //a diff drive can only face along its path
    if constexpr (!Drive::HOLONOMIC) {
        m_params.heading = HeadingMode::TANGENT;
    }
}

//build reference trajectory
template<class Drive, std::size_t F>
typename MPCReference<Drive, F>::ZRef MPCReference<Drive, F>::build(const Pose& currentPose, const ALS_Path& path, std::size_t idx, bool reverse, const VecM& omega) {
    const std::vector<Sample>& samples = path.getSamples();
    const double s_end = samples.back().s;
    constexpr double INF = std::numeric_limits<double>::infinity();

    //physical consts and refrence-gen limits
    const double h = m_params.h;
    const double A_MAX = m_params.a_max; // in/s^2, reference accel/decel
    const double V_REF_MIN = m_params.v_ref_min; // in/s, floor so reference can pull away from rest
    //top speed each group plans for: 90% of its 12 V steady state (rad/s)
    std::array<double, N> wheel_max{};
    for (std::size_t j = 0; j < N; j++) {
        wheel_max[j] = 0.9 * 12.0 * m_params.b[j] / m_params.a[j];
    }
    // max feasible heading rate derate to 90%: the slowest group that spins to rotate the robot sets it
    double W_REF_MAX = INF;
    for (std::size_t j = 0; j < N; j++) {
        if (std::abs(m_G[j][2]) > 1e-9) {
            W_REF_MAX = std::min(W_REF_MAX, wheel_max[j] / std::abs(m_G[j][2]));
        }
    }
    W_REF_MAX *= 0.9;

    //latch the heading HOLD keeps on the first build after reset()
    if (!m_have_hold) {
        m_hold_theta = currentPose.theta;
        m_have_hold = true;
    }
    //target heading at a path point before rate limiting
    const HeadingMode mode = m_params.heading;
    const double hold_theta = m_hold_theta;
    auto headingAt = [mode, reverse, hold_theta](const InterpSample& smp) -> double {
        if (mode == HeadingMode::HOLD) {
            return hold_theta;
        }
        //TANGENT: if driving backwards, face opposite to path tangent
        return reverse ? utils::wrapAngle(smp.theta + M_PI) : smp.theta;
    };

    //speed cap at arc length sq(sQuery): every group's speed along the path plus the turn the heading needs stays under its top speed
    auto capAt = [&](double sq) -> double {
        InterpSample a = path.sampleAtArcLength(sq);
        //the pt ~3in ahead (clamped)
        InterpSample b = path.sampleAtArcLength(std::min(sq + 3.0, s_end));
        double ds = std::max(1e-3, b.s - a.s);
        //heading change per inch the robot has to make (path curvature when following the tangent)
        double head_a = headingAt(a);
        double kappa_h = std::abs(utils::wrapAngle(headingAt(b) - head_a)) / ds;
        //path direction in the body frame (forward, left), Pose convention headings
        double rel = a.theta - head_a;
        double dx = std::cos(rel);
        double dy = -std::sin(rel);
        double cap = INF;
        for (std::size_t j = 0; j < N; j++) {
            //group speed per in/s of travel
            double per_v = std::abs(m_G[j][0] * dx + m_G[j][1] * dy) + std::abs(m_G[j][2]) * kappa_h;
            if (per_v > 1e-9) {
                cap = std::min(cap, wheel_max[j] / per_v);
            }
        }
        //X and H drives: nothing else slows them in corners, so cap the cornering accel v^2 * kappa
        if constexpr (Drive::HOLONOMIC) {
            double kappa_p = std::abs(utils::wrapAngle(b.theta - a.theta)) / ds;
            if (kappa_p > 1e-6) {
                cap = std::min(cap, std::sqrt(m_params.a_lat_max / kappa_p));
            }
        }
        return cap;
    };

    //braking speed aware cap: look ahead over the current stopping distance and make sure we could deaccelerate
    auto brakingCapAt = [&](double sq, double v_now) -> double {
        //start with local curveture cap
        double cap = capAt(sq);
        //stoping distance for v_now plus some margin
        double brake_dist = (v_now * v_now) / (2.0 * A_MAX) + 2.0;
        //scan ahead in 2 inch steps
        for (double d = 2.0; d <= brake_dist; d += 2.0) {
            double s_ahead = sq + d;
            //stop scanning past the end
            if (s_ahead >= s_end) {
                break;
            }
            //future cap at distance d
            double c = capAt(s_ahead);
            //max speed now so that decelerating at A_MAX over d reaches c
            cap = std::min(cap, std::sqrt(c * c + 2.0 * A_MAX * d));
        }
        //enforce full stop at the path end
        double d_end = std::max(0.0, s_end - sq);
        cap = std::min(cap, std::sqrt(2.0 * A_MAX * d_end));
        return cap;
    };

    //measured speed from the body twist of the measured group speeds
    double vx_meas = 0.0;
    double vy_meas = 0.0;
    for (std::size_t j = 0; j < N; j++) {
        vx_meas += m_K[0][j] * omega(j);
        vy_meas += m_K[1][j] * omega(j);
    }
    double v_meas = std::hypot(vx_meas, vy_meas);
    double v;
    if (m_v_ref < 0.0) {
        //first run, get val from measurements (floored)
        v = std::max(v_meas, V_REF_MIN);
    }
    else {
        //continue from last cycle's ref. but keep it within 25 in/s of the measured speed
        v = std::clamp(m_v_ref, v_meas - 25.0, v_meas + 25.0);
        v = std::max(v, V_REF_MIN);
    }

    //start rolling out from closest path pt
    double s = samples[idx].s;
    //prev stages unrawped ref theta
    double prev_theta = 0.0;
    bool have_prev = false;

    ZRef z;
    //gen F+1 reference stages
    for (std::size_t i = 0; i <= F; i++) {
        //pose at current rollout arc length
        InterpSample ref = path.sampleAtArcLength(s);
        double ref_theta = headingAt(ref);
        if (!have_prev) {
            //first stage: unrwap rel to robot's actual heading so MPC sees small heading error
            ref_theta = currentPose.theta + utils::wrapAngle(ref_theta - currentPose.theta);
        }
        else {
            //subsequent stages: rate-limit heading change to the feasible turn-rate
            double dth = utils::wrapAngle(ref_theta - prev_theta);
            //max heading change per step
            double dth_max = W_REF_MAX * h;
            dth = std::clamp(dth, -dth_max, dth_max);
            ref_theta = prev_theta + dth;
        }
        prev_theta = ref_theta;
        have_prev = true;

        //write stage i ref
        z(i * r_states + 0) = ref.x;
        z(i * r_states + 1) = ref.y;
        z(i * r_states + 2) = ref_theta;

        //advance velocity orofile one step lower floor in last 6 in so it can settle
        double v_floor = (s_end - s < 6.0) ? 1.0 : V_REF_MIN;
        //speed limit at this pt
        double v_cap = brakingCapAt(s, v);
        //accel toward cap at max accel
        double v_target = std::min(v_cap, v + A_MAX * h);
        // but decel no faster than accel max (floored)
        v = std::max({v_target, v - A_MAX * h, v_floor});
        //advance arc length by one step
        s += v * h;
        if (s > s_end) {
            //clamp rollout at path end
            s = s_end;
        }

        //persist the 1st stage speed as seed for next cycle
        if (i == 0) {
            m_v_ref = v;
        }
    }
    return z;
}

//explicit instantiation for every drive (must match MPCController)
template class MPCReference<kinematics::DiffDrive, MPC_PREDICTION_HORIZON>;
template class MPCReference<kinematics::XDrive, MPC_PREDICTION_HORIZON>;
template class MPCReference<kinematics::HDrive, MPC_PREDICTION_HORIZON>;
