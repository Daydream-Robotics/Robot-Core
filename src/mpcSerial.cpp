#include "mpcSerial.hpp"
#include "subsystems.hpp"
#include "pros/rtos.hpp"
#include "pros/misc.hpp"
#include <cmath>
#include <algorithm>
#include "pathFollower.hpp"
#include "sd_card_logging.hpp"
#include "constants.h"

//store sample period and gear ratio
MPCSerial::Params::Params(double frequency, double ratio, double trackWidth, double A, double B, double aMax, double vMin)
    : h(frequency), gear_ratio(ratio), track_width(trackWidth), a(A), b(B), a_max(aMax), v_ref_min(vMin) {}

//initialize serial link (8 kB buffer, binary mode) and configure vex serial
MPCSerial::MPCSerial(const Params& params)
    : m_params(params), serial(8192, SerialProtocol::Mode::BINARY)
{
    SerialProtocol::setUpVexSerial();
}

//wrap angle to (-pi, pi]
static inline double wrapAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

//find closest point on spline for given arc length
MPCSerial::InterpSample MPCSerial::sampleAtArcLength(const std::vector<Sample>& samples, double sQuery) {
    InterpSample result;
    //if empty path: return zeros
    if (samples.empty()) {
        return result;
    }

    //clamp to start
    if (sQuery <= samples.front().s) {
        result.x = samples.front().x;
        result.y = samples.front().y;
        result.theta = samples.front().heading;
        result.v = samples.front().v;
        result.s = samples.front().s;
        return result;
    }
    //clamp to end
    if (sQuery >= samples.back().s) {
        result.x = samples.back().x;
        result.y = samples.back().y;
        result.theta = samples.back().heading;
        result.v = samples.back().v;
        result.s = samples.back().s;
        return result;
    }

    // binary search for bracket [left, right]
    std::size_t left = 0;
    std::size_t right = samples.size() - 1;
    while (left + 1 < right) {
        std::size_t mid = left + (right - left) / 2;
        if (samples[mid].s <= sQuery) {
            left = mid;
        } else {
            right = mid;
        }
    }

    //linearly interpolate between bracketing samples a & b
    const Sample& a = samples[left];
    const Sample& b = samples[right];
    //arc length gap btwn samples
    double ds = b.s - a.s;
    //normalized interpolation
    double alpha = (std::abs(ds) < 1e-12) ? 0.0 : (sQuery - a.s) / ds;
    //shoretest angle diff
    double dtheta = wrapAngle(b.heading - a.heading);

    //interpolate x, y and theta
    result.x = a.x + alpha * (b.x - a.x);
    result.y = a.y + alpha * (b.y - a.y);
    result.theta = wrapAngle(a.heading + alpha * dtheta);
    // interpolated reference speed and arc length
    result.v = a.v + alpha * (b.v - a.v);
    result.s = a.s + alpha * ds;

    return result;
}



//pack current state + build reference trajectory into update packet
/* (the reference is gend by looking forward along the path where 
speed is limited by curvature by a lookahead constraint 
by an accel/decel limit, and forced to zero at the path end. 
headings are rate-limited so the ref never asks for an infeasible turn rate) */
MPCSerial::MPCUpdatePacket MPCSerial::buildUpdatePacket(const Pose& currentPose, const std::vector<Sample>& samples, std::size_t idx, PathFlag flag, double omega_L, double omega_R, double V_battery, double I_total) {
    //current measure state downcast to a float
    MPCUpdatePacket p{};
    p.pose_x = static_cast<float>(currentPose.x);
    p.pose_y = static_cast<float>(currentPose.y);
    p.pose_theta = static_cast<float>(currentPose.theta);
    p.omega_L = static_cast<float>(omega_L);
    p.omega_R = static_cast<float>(omega_R);
    p.V_battery = static_cast<float>(V_battery);
    p.I_total = static_cast<float>(I_total);

    //physical consts and refrence-gen limits
    const double h = m_params.h;
    const double r = DRIVE_WHEEL_DIAMETER_INCHES / 2.0;
    const double TRACK_WIDTH = m_params.track_width; // in
    const double A_MAX = m_params.a_max; // in/s^2, reference accel/decel
    const double V_REF_MIN = m_params.v_ref_min; // in/s, floor so reference can pull away from rest
    const double V_STRAIGHT_MAX = 0.9 * (m_params.b / m_params.a) * 12.0 * r;
    // max feasible heading rate derate to 90%
    const double W_REF_MAX = 0.9 * (2.0 * r * (0.9 * (m_params.b / m_params.a) * 12.0)) / TRACK_WIDTH;

    //curvature based speed cap at arc length sq(sQuery): est local kappa from heading change then limit speed so the outer wheel never exceeds V_STRAIGHT_MAX
    auto capAt = [&](double sq) -> double {
        InterpSample a = sampleAtArcLength(samples, sq);
        //the pt ~3in ahead (clamped)
        InterpSample b = sampleAtArcLength(samples,std::min(sq + 3.0, samples.back().s));
        double ds = std::max(1e-3, b.s - a.s);
        //curvature
        double kappa = std::abs(wrapAngle(b.theta - a.theta)) / ds;
        //return outer wheel limit
        return V_STRAIGHT_MAX / (1.0 + kappa * TRACK_WIDTH / 2.0);
    };

    //braking speed aware cap: ook ahead over the current stopping distance and make sure we could deaccelerate
    auto brakingCapAt = [&](double sq, double v_now) -> double {
        //start with local curveture cap
        double cap = capAt(sq);
        //stoping distance for v_now plus some margin
        double brake_dist = (v_now * v_now) / (2.0 * A_MAX) + 2.0;
        //scan ahead in 2 inch steps
        for (double d = 2.0; d <= brake_dist; d += 2.0) {
            double s_ahead = sq + d;
            //stop scanning past the end
            if (s_ahead >= samples.back().s) {
                break;
            }
            //future cap at distance d
            double c = capAt(s_ahead);
            //max speed now so that decelerating at A_MAX over d reaches c
            cap = std::min(cap, std::sqrt(c * c + 2.0 * A_MAX * d));
        }
        //enforce full stop at the path end
        double d_end = std::max(0.0, samples.back().s - sq);
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

    //gen F+1 reference stages
    for (std::size_t i = 0; i <= F; i++) {
        //pose at current rollout arc length
        InterpSample ref = sampleAtArcLength(samples, s);
        double ref_theta = ref.theta;
        if (flag == PathFlag::REVERSE) {
            //if driving backwards, face opposite to path tangent
            ref_theta = wrapAngle(ref_theta + M_PI);
        }
        if (!have_prev) {
            //first stage: unrwap rel to robot's actual heading so MPC sees small heading error
            ref_theta = currentPose.theta + wrapAngle(ref_theta - currentPose.theta);
        } 
        else {
            //subsequent stages: rate-limit heading change to the feasible turn-rate
            double dth = wrapAngle(ref_theta - prev_theta);
            //max heading change per step
            double dth_max = W_REF_MAX * h;
            dth = std::clamp(dth, -dth_max, dth_max);
            ref_theta = prev_theta + dth;          
        }
        prev_theta = ref_theta;
        have_prev = true;

        //write stage i ref into packet array
        std::size_t base = i * 3;
        p.z_desired[base + 0] = static_cast<float>(ref.x);
        p.z_desired[base + 1] = static_cast<float>(ref.y);
        p.z_desired[base + 2] = static_cast<float>(ref_theta);

        //advance velocity orofile one step lower floor in last 6 in so it can settle
        double v_floor = (samples.back().s - s < 6.0) ? 1.0 : V_REF_MIN;
        //speed limit at this pt
        double v_cap = brakingCapAt(s, v);
        //accel toward cap at max accel
        double v_target = std::min(v_cap, v + A_MAX * h);
        // but decel no faster than accel max (floored)
        v = std::max({v_target, v - A_MAX * h, v_floor});
        //advance arc length by one step
        s += v * h;
        if (s > samples.back().s) {
            //clamp rollout at path end
            s = samples.back().s;
        }

        //persist the 1st stage speed as seed for next cycle
        if (i == 0) {
            m_v_ref = v;
        }
    }
    return p;
}

//public compute: read sensors then call private compute
WheelVelocities MPCSerial::compute(const Pose& currentPose, const ALS_Path& path, std::size_t& closestIdx, PathFlag flag) {
    // convert motor RPM to rad/s at wheels
    double raw_omega_L = leftMotors.get_actual_velocity(0) * 2.0 * M_PI / 60.0;
    double raw_omega_R = rightMotors.get_actual_velocity(0) * 2.0 * M_PI / 60.0;
    double wheel_omega_L = raw_omega_L * m_params.gear_ratio;
    double wheel_omega_R = raw_omega_R * m_params.gear_ratio;

    //get battery state in V and A
    double V_battery = pros::battery::get_voltage() / 1000.0;
    double I_total = pros::battery::get_current() / 1000.0; 

    return compute(currentPose, path, closestIdx, flag, wheel_omega_L, wheel_omega_R, V_battery, I_total);
}

// private compute: send to microcontroller, receive optimal voltages
//builds packet, sends via serial, blocks for response from microcontroller
WheelVelocities MPCSerial::compute(const Pose& currentPose, const ALS_Path& path, std::size_t& closestIdx, PathFlag flag, double omega_L, double omega_R, double V_battery, double I_total) {
    //pack current state + F-stage reference trajectory
    MPCUpdatePacket req = buildUpdatePacket(currentPose, path.getSamples(), closestIdx, flag, omega_L, omega_R, V_battery, I_total);
    // flush stdout so any pending debug prints don't interfere with serial traffic
    fflush(stdout);
    std::fflush(stdout);

    //send update packet
    pros::lcd::print(0, "send");
    serial.send(static_cast<uint16_t>(SerialProtocol::PacketType::MPC_UPDATE), req);
    pros::lcd::print(0, "sent");

    //block until we get a control response
    std::optional<MPCControlPacket> res = serial.receive<MPCControlPacket>(static_cast<uint16_t>(SerialProtocol::PacketType::MPC_CONTROL));
    // debug: indicate receive
    pros::lcd::print(0, "recieve");

    // timeout or bad packet: zero voltage
    if (!res.has_value()) {
        return {0.0, 0.0, ControlMode::INPUT_VOLTAGE}; 
    }
    
    // convert V to mV and return
    return {
        res->V_left * 1000.0, 
        res->V_right * 1000.0,
        ControlMode::INPUT_VOLTAGE
    };
}


//estimate motor time constant a
//fits the first-order motor time constant a from a step response. to:
//model: a = -ln(1 - omega/omega_ss) / t: it averages the per-sample estimates, 
//discarding samples too close to rest
double MPCSerial::estimateA(const std::vector<double>& time, const std::vector<double>& omega, double omega_ss) {
    double sum = 0.0;
    int count = 0;
    for (std::size_t i = 0; i < omega.size(); i++) {
         //remaining fraction of transient
        double ratio = 1.0 - (omega[i] / omega_ss);
        //keep only the middle of the transient
        if (ratio <= 0.05 || ratio >= 0.95) {
            continue;
        }

        // per-sample estimate of a
        double a_i = -std::log(ratio) / time[i];
        //reject NaN/inf and unphysical fits
        if (std::isfinite(a_i) && a_i > 0.0) {
            sum += a_i;
            count++;
        }
    }
    //no valid samples: return 0
    if (count == 0) {
        return 0.0;
    }
    //avg
    return sum / count;
}

//estimate motor gain b
//fits the motor input gain b from steady state: at steady state omega_dot = 0, 
//so b = a * omega_ss / V.
double MPCSerial::estimateB(double a, double omega_ss, double voltage) {
    return (a * omega_ss) / voltage;
}

//single step response test: applies constant voltage, records omega(t), fits a and b
void MPCSerial::runSingleIdentificationTest(int voltage, double& out_a, double& out_b, double gear_ratio) {
    std::vector<double> time;
    std::vector<double> omega;

    // settle to zero
    pros::delay(1000);
    int start = pros::millis();

    // apply voltage
    leftMotors.move_voltage(voltage);

    //record omega(t) for 2 seconds
    while (pros::millis() - start < 2000) {
        double t = (pros::millis() - start) / 1000.0;
        double w = leftMotors.get_actual_velocity(0) * 2.0 * M_PI / 60.0;
        time.push_back(t);
        omega.push_back(w);
        pros::delay(10);
    }

    //stop motor
    leftMotors.move_voltage(0);

    //estimate steady-state omega
    double omega_ss_motor = 0;
    int n_last = std::min<int>(50, omega.size());
    for (std::size_t i = omega.size() - n_last; i < omega.size(); i++) {
        omega_ss_motor += omega[i];
    }
    omega_ss_motor /= n_last;

    //fit a from the transient 
    out_a = estimateA(time, omega, omega_ss_motor);
    //convert steady state to wheel-side for the gain fit
    double omega_ss_wheel = omega_ss_motor * gear_ratio;
    //fit b with voltage converted from mV to V
    out_b = estimateB(out_a, omega_ss_wheel, voltage / 1000.0);

    printf("\n");
    printf("Voltage: %d mV\n", voltage);
    printf("omega_ss_motor: %f rad/s\n", omega_ss_motor);
    printf("omega_ss_wheel: %f rad/s\n", omega_ss_wheel);
    printf("a: %f\n", out_a);
    printf("b: %f\n", out_b);
}

//full motor identification: runs multiple voltage steps, averages fitted parameters
void MPCSerial::identifyMotorModel(double gear_ratio) {
    double sum_a = 0.0;
    double sum_b = 0.0;
    int count = 0;

    //helper: run one test at the given voltage and accumulate its fit
    auto identify = [&](int voltage) {
        double a = 0.0, b = 0.0;
        runSingleIdentificationTest(voltage, a, b, gear_ratio);
        sum_a += a;
        sum_b += b;
        count++;
    };

    // sweep voltages
    identify(5000);
    identify(7500);
    identify(10000);
    identify(12000);

    //report averages
    double avg_a = sum_a / count;
    double avg_b = sum_b / count;

    printf("\n====================\n");
    printf("AVERAGE MODEL\n");
    printf("====================\n");
    printf("a: %f\n", avg_a);
    printf("b: %f\n", avg_b);

    pros::lcd::print(0, "AVERAGE MODEL");
    pros::lcd::print(1, "a: %.4f", avg_a);
    pros::lcd::print(2, "b: %.4f", avg_b);
}
