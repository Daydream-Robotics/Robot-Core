#include "motion/control/mpcSerial.hpp"
#include "config/subsystems.hpp"
#include "pros/rtos.hpp"
#include "pros/misc.hpp"
#include <cmath>
#include <algorithm>
#include "motion/pathFollower.hpp"
#include "utils/sd_card_logging.hpp"
#include "config/constants.h"

//the brain's odometry wheel size and the MPC's shared wheel radius must agree
static_assert(DRIVE_WHEEL_DIAMETER_INCHES == 2.0 * mpc_config::WHEEL_RADIUS, "DRIVE_WHEEL_DIAMETER_INCHES doesn't match mpc_config::WHEEL_RADIUS");

//store gear ratio and reference-gen params
MPCSerial::Params::Params(double ratio, double trackWidth)
    : gear_ratio(ratio), ref(trackWidth) {}

//initialize serial link (8 kB buffer, binary mode) and configure vex serial
MPCSerial::MPCSerial(const Params& params)
    : m_params(params), serial(8192, SerialProtocol::Mode::BINARY)
{
    SerialProtocol::setUpVexSerial();
}

//pack current state + build reference trajectory into update packet (rollout shared with the Nano, see referenceRollout.hpp)
MPCUpdatePacket MPCSerial::buildUpdatePacket(const Pose& currentPose, const ALS_Path& path, std::size_t idx, PathFlag flag, double omega_L, double omega_R, double V_battery, double I_total) {
    //current measure state downcast to a float
    MPCUpdatePacket p{};
    p.protocol_version = MPC_PROTOCOL_VERSION;
    p.pose_x = static_cast<float>(currentPose.x);
    p.pose_y = static_cast<float>(currentPose.y);
    p.pose_theta = static_cast<float>(currentPose.theta);
    p.omega_L = static_cast<float>(omega_L);
    p.omega_R = static_cast<float>(omega_R);
    p.V_battery = static_cast<float>(V_battery);
    p.I_total = static_cast<float>(I_total);
    //voltage the motors are getting now (mV -> V), so a reset or dropped packet can't make the Nano rate-limit from the wrong voltage
    //PROS_ERR becomes NaN, which the Nano rejects (failed solve -> 0 V)
    const std::int32_t mV_left = leftMotors.get_voltage(0);
    const std::int32_t mV_right = rightMotors.get_voltage(0);
    p.V_left_applied = (mV_left == PROS_ERR) ? NAN : static_cast<float>(mV_left / 1000.0);
    p.V_right_applied = (mV_right == PROS_ERR) ? NAN : static_cast<float>(mV_right / 1000.0);
    p.track_width = static_cast<float>(m_params.ref.track_width);
    p.reset_controller = m_reset_pending ? 1 : 0;

    //F+1 reference poses, built in an aligned local array then copied in:
    //binding a float& to the packed packet's array could make unaligned VFP stores, which fault on the brain's Cortex-A9
    const std::vector<Sample>& samples = path.getSamples();
    auto sampleAt = [&path](double s) { return path.sampleAtArcLength(s); };
    float z[(F + 1) * 3];
    mpc_ref::rollout<F>(m_params.ref, sampleAt, samples[idx].s, samples.back().s, currentPose.theta, flag == PathFlag::REVERSE, omega_L, omega_R, m_v_ref, z);
    for (std::size_t i = 0; i < (F + 1) * 3; i++) {
        p.z_desired[i] = z[i];
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
    MPCUpdatePacket req = buildUpdatePacket(currentPose, path, closestIdx, flag, omega_L, omega_R, V_battery, I_total);
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
    //brain and Nano built from different packet versions: don't trust the voltages
    if (res->protocol_version != MPC_PROTOCOL_VERSION) {
        pros::lcd::print(1, "MPC protocol mismatch: brain v%d, Nano v%d", MPC_PROTOCOL_VERSION, res->protocol_version);
        return {0.0, 0.0, ControlMode::INPUT_VOLTAGE};
    }
    //the Nano answered a packet, so it has seen any pending reset
    m_reset_pending = false;
    
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
