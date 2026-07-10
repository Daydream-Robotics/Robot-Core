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
MPCSerial::Params::Params(double frequency, double ratio)
    : h(frequency), gear_ratio(ratio) {}

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

    //linearly interpolate between bracketing samples
    const Sample& a = samples[left];
    const Sample& b = samples[right];
    double ds = b.s - a.s;
    double alpha = (std::abs(ds) < 1e-12) ? 0.0 : (sQuery - a.s) / ds;
    double dtheta = wrapAngle(b.heading - a.heading);

    result.x = a.x + alpha * (b.x - a.x);
    result.y = a.y + alpha * (b.y - a.y);
    result.theta = wrapAngle(a.heading + alpha * dtheta);
    result.v = a.v + alpha * (b.v - a.v);
    result.s = a.s + alpha * ds;

    return result;
}

void computeVelocityProfile(std::vector<Sample>& samples) {
    if (samples.size() < 2) return;

    const double r = DRIVE_WHEEL_DIAMETER_INCHES / 2.0;
    const double L = 10.5;
    const double omega_wheel_max = 0.9 * (138.9554 / 30.0308) * 12.0;  
    const double v_straight_max  = omega_wheel_max * r;
    const double A_MAX = 60.0;  

    const std::size_t N = samples.size();

    for (std::size_t i = 0; i + 1 < N; i++) {
        double ds = samples[i + 1].s - samples[i].s;
        if (ds < 1e-9) { 
            samples[i].v = v_straight_max; continue; 
        }
        double kappa = std::abs(wrapAngle(samples[i + 1].heading - samples[i].heading)) / ds;
        samples[i].v = v_straight_max / (1.0 + kappa * L / 2.0);
    }
    samples[N - 1].v = 0.0; 

    for (std::size_t i = N - 1; i-- > 0; ) {
        double ds = samples[i + 1].s - samples[i].s;
        samples[i].v = std::min(samples[i].v,
            std::sqrt(samples[i + 1].v * samples[i + 1].v + 2.0 * A_MAX * ds));
    }
    for (std::size_t i = 1; i < N; i++) {
        double ds = samples[i].s - samples[i - 1].s;
        samples[i].v = std::min(samples[i].v,
            std::sqrt(samples[i - 1].v * samples[i - 1].v + 2.0 * A_MAX * ds));
    }
}

//pack current state + build reference trajectory into update packet
// pack current state + build reference trajectory into update packet
MPCSerial::MPCUpdatePacket MPCSerial::buildUpdatePacket(
    const Pose& currentPose, const std::vector<Sample>& samples, 
    std::size_t idx, PathFlag flag, double omega_L, double omega_R, 
    double V_battery, double I_total) 
{
    MPCUpdatePacket p{};

    p.pose_x = static_cast<float>(currentPose.x);
    p.pose_y = static_cast<float>(currentPose.y);
    p.pose_theta = static_cast<float>(currentPose.theta);
    p.omega_L = static_cast<float>(omega_L);
    p.omega_R = static_cast<float>(omega_R);
    p.V_battery = static_cast<float>(V_battery);
    p.I_total = static_cast<float>(I_total);

    // fixed spatial step between horizon stages
    // 1.0 inch per stage × F=15 stages = 15 inch preview
    // matches robot's expected travel per horizon at ~50 in/s
    constexpr double DS = 1.0;   // inches per horizon stage
    
    double s = samples[idx].s;
    for (std::size_t i = 0; i < F; i++) {
        s += DS;
        if (s > samples.back().s) {
            s = samples.back().s;
        }

        InterpSample ref = sampleAtArcLength(samples, s);

        double refTheta = ref.theta;
        if (flag == PathFlag::REVERSE) {
            refTheta = wrapAngle(refTheta + M_PI);
        }

        std::size_t base = i * 3;
        p.z_desired[base + 0] = static_cast<float>(ref.x);
        p.z_desired[base + 1] = static_cast<float>(ref.y);
        p.z_desired[base + 2] = static_cast<float>(refTheta);
    }
    return p;
}

//public compute: read sensors, call private compute
WheelVelocities MPCSerial::compute(const Pose& currentPose, const ALS_Path& path, std::size_t& closestIdx, PathFlag flag) {

    double u_l = wheelVelocities.left / 1000.0;
    double u_r = wheelVelocities.right / 1000.0;

    double omega_motor_L = (138.9554 / 30.0308) * u_l;
    double omega_motor_R = (138.9554 / 30.0308) * u_r;

    double wheel_omega_L = omega_L;
    double wheel_omega_R = omega_R;

    //convert motor RPM to rad/s at wheels
    // double raw_omega_L = leftMotors.get_actual_velocity(0) * 2.0 * M_PI / 60.0;
    // double raw_omega_R = rightMotors.get_actual_velocity(0) * 2.0 * M_PI / 60.0;
    // double wheel_omega_L = raw_omega_L * m_params.gear_ratio;
    // double wheel_omega_R = raw_omega_R * m_params.gear_ratio;

    //get battery state in V and A
    double V_battery = pros::battery::get_voltage() / 1000.0;
    double I_total = pros::battery::get_current() / 1000.0; 

    return compute(currentPose, path, closestIdx, flag, wheel_omega_L, wheel_omega_R, V_battery, I_total);
}

// private compute: send to microcontroller, receive optimal voltages
//builds packet, sends via serial, blocks for response from microcontroller
WheelVelocities MPCSerial::compute(const Pose& currentPose, const ALS_Path& path, std::size_t& closestIdx, PathFlag flag, double omega_L, double omega_R, double V_battery, double I_total) {
    //pack current state + F-stage reference trajectory
    // Logger::getInstance().init("/usd/log.txt");
    MPCUpdatePacket req = buildUpdatePacket(currentPose, path.getSamples(), closestIdx, flag, omega_L, omega_R, V_battery, I_total);
    fflush(stdout);
    std::fflush(stdout);

    //debug: indicate send start
    serial.send(static_cast<uint16_t>(SerialProtocol::PacketType::MPC_UPDATE), req);
    // debug: indicate send complete
    pros::lcd::print(0, "sent");
    LOG("sent");

    //block for optimal control response
    std::optional<MPCControlPacket> res = serial.receive<MPCControlPacket>(static_cast<uint16_t>(SerialProtocol::PacketType::MPC_CONTROL));
    // debug: indicate receive
    pros::lcd::print(0, "recieve");
    LOG("receive");

    // timeout or bad packet: zero voltage
    if (!res.has_value()) {
        return {0.0, 0.0, ControlMode::INPUT_VOLTAGE}; 
    }
    
    // convert V to mV
    return {
        res->V_left * 1000.0, 
        res->V_right * 1000.0,
        ControlMode::INPUT_VOLTAGE
    };
}


//estimate motor time constant a
double MPCSerial::estimateA(const std::vector<double>& time,
                             const std::vector<double>& omega,
                             double omega_ss) {
    double sum = 0.0;
    int count = 0;
    for (std::size_t i = 0; i < omega.size(); i++) {
        double ratio = 1.0 - (omega[i] / omega_ss);
        if (ratio <= 0.05 || ratio >= 0.95) {
            continue;
        }

        double a_i = -std::log(ratio) / time[i];
        if (std::isfinite(a_i) && a_i > 0.0) {
            sum += a_i;
            count++;
        }
    }
    if (count == 0) {
        return 0.0;
    }
    return sum / count;
}

//estimate motor gain b
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

    //record for 2 seconds
    while (pros::millis() - start < 2000) {
        double t = (pros::millis() - start) / 1000.0;
        double w = leftMotors.get_actual_velocity(0) * 2.0 * M_PI / 60.0;
        time.push_back(t);
        omega.push_back(w);
        pros::delay(10);
    }

    //stop motor
    leftMotors.move_voltage(0);
    //fit model
    double omega_ss_motor = 0;
    int n_last = std::min<int>(50, omega.size());
    for (std::size_t i = omega.size() - n_last; i < omega.size(); i++) {
        omega_ss_motor += omega[i];
    }
    omega_ss_motor /= n_last;
    out_a = estimateA(time, omega, omega_ss_motor);
    double omega_ss_wheel = omega_ss_motor * gear_ratio;
    //normalize voltage to [0,1] range
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

    //helper: run test and accumulate
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
