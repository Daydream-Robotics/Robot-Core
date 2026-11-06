#include "motion/control/mpcReference.hpp"
#include "utils/mathHelpers.hpp"
#include <algorithm>
#include <cmath>

template<std::size_t F>
MPCReference<F>::MPCReference(const Params& params)
    : m_params(params) {}

//build reference trajectory
template<std::size_t F>
typename MPCReference<F>::ZRef MPCReference<F>::build(const Pose& currentPose, const ALS_Path& path, std::size_t idx, bool reverse, double omega_L, double omega_R) {
    const std::vector<Sample>& samples = path.getSamples();
    const double s_end = samples.back().s;

    //physical consts and refrence-gen limits
    const double h = m_params.h;
    const double r = m_params.r;
    const double TRACK_WIDTH = m_params.track_width; // in
    const double A_MAX = m_params.a_max; // in/s^2, reference accel/decel
    const double V_REF_MIN = m_params.v_ref_min; // in/s, floor so reference can pull away from rest
    const double V_STRAIGHT_MAX = 0.9 * (m_params.b / m_params.a) * 12.0 * r;
    // max feasible heading rate derate to 90%
    const double W_REF_MAX = 0.9 * (2.0 * r * (0.9 * (m_params.b / m_params.a) * 12.0)) / TRACK_WIDTH;

    //curvature based speed cap at arc length sq(sQuery): est local kappa from heading change then limit speed so the outer wheel never exceeds V_STRAIGHT_MAX
    auto capAt = [&](double sq) -> double {
        InterpSample a = path.sampleAtArcLength(sq);
        //the pt ~3in ahead (clamped)
        InterpSample b = path.sampleAtArcLength(std::min(sq + 3.0, s_end));
        double ds = std::max(1e-3, b.s - a.s);
        //curvature
        double kappa = std::abs(utils::wrapAngle(b.theta - a.theta)) / ds;
        //return outer wheel limit
        return V_STRAIGHT_MAX / (1.0 + kappa * TRACK_WIDTH / 2.0);
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

    //meausred fwd speed for average wheel omega
    double v_meas = std::abs(0.5 * (omega_L + omega_R) * r);
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
        double ref_theta = ref.theta;
        if (reverse) {
            //if driving backwards, face opposite to path tangent
            ref_theta = utils::wrapAngle(ref_theta + M_PI);
        }
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

//explicit instantiation
template class MPCReference<MPC_PREDICTION_HORIZON>;
