#include "mpcSerial.hpp"
#include "subsystems.hpp"
#include "pros/rtos.hpp"
#include "pros/misc.hpp"
#include <cmath>
#include <algorithm>

MPCSerial::Params::Params(double frequency, double ratio)
    : h(frequency), gear_ratio(ratio) {}

MPCSerial::MPCSerial(const Params& params)
    : m_params(params), serial(8192, SerialProtocol::Mode::BINARY)
{
    SerialProtocol::setUpVexSerial();
}

static inline double wrapAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

MPCSerial::InterpSample MPCSerial::sampleAtArcLength(const std::vector<Sample>& samples, double sQuery) {
    InterpSample result;
    if (samples.empty()) {
        return result;
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
    std::size_t right = samples.size() - 1;

    while (left + 1 < right) {
        std::size_t mid = left + (right - left) / 2;
        if (samples[mid].s <= sQuery) {
            left = mid;
        } else {
            right = mid;
        }
    }
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

MPCSerial::MPCUpdatePacket MPCSerial::buildUpdatePacket(
    const Pose& currentPose, 
    const std::vector<Sample>& samples, 
    std::size_t idx, 
    double omega_L, 
    double omega_R, 
    double V_battery, 
    double I_total
) {
    MPCUpdatePacket p{};

    p.pose_x = static_cast<float>(currentPose.x);
    p.pose_y = static_cast<float>(currentPose.y);
    p.pose_theta = static_cast<float>(currentPose.theta);

    p.omega_L = static_cast<float>(omega_L);
    p.omega_R = static_cast<float>(omega_R);
    p.V_battery = static_cast<float>(V_battery);
    p.I_total = static_cast<float>(I_total);

    double s = samples[idx].s;
    for (std::size_t i = 0; i < F; i++) {
        InterpSample curr = sampleAtArcLength(samples, s);
        double v = std::max(curr.v, 0.0);
        s += v * m_params.h;
        if (s > samples.back().s) {
            s = samples.back().s;
        }
        InterpSample ref = sampleAtArcLength(samples, s);
        std::size_t base = i * 3;
        p.z_desired[base + 0] = static_cast<float>(ref.x);
        p.z_desired[base + 1] = static_cast<float>(ref.y);
        p.z_desired[base + 2] = static_cast<float>(ref.theta);
    }
    return p;
}

WheelVelocities MPCSerial::compute(const Pose& currentPose, const ALS_Path& path, std::size_t& closestIdx) {
    double raw_omega_L = leftMotors.get_actual_velocity(0) * 2.0 * M_PI / 60.0;
    double raw_omega_R = rightMotors.get_actual_velocity(0) * 2.0 * M_PI / 60.0;

    double wheel_omega_L = raw_omega_L * m_params.gear_ratio;
    double wheel_omega_R = raw_omega_R * m_params.gear_ratio;

    double V_battery = pros::battery::get_voltage() / 1000.0;
    double I_total = pros::battery::get_current() / 1000.0; 

    return compute(currentPose, path, closestIdx, wheel_omega_L, wheel_omega_R, V_battery, I_total);
}

WheelVelocities MPCSerial::compute(
    const Pose& currentPose, 
    const ALS_Path& path, 
    std::size_t& closestIdx, 
    double omega_L, 
    double omega_R, 
    double V_battery, 
    double I_total
) {
    MPCUpdatePacket req = buildUpdatePacket(currentPose, path.getSamples(), closestIdx, omega_L, omega_R, V_battery, I_total);
    
    serial.send(static_cast<uint16_t>(SerialProtocol::PacketType::MPC_UPDATE), req);
    
    std::optional<MPCControlPacket> res = serial.receive<MPCControlPacket>(static_cast<uint16_t>(SerialProtocol::PacketType::MPC_CONTROL));
    if (!res.has_value()) {
        return {0.0, 0.0, ControlMode::INPUT_VOLTAGE}; 
    }
    
    return {
        res->V_left * 1000.0, 
        res->V_right * 1000.0,
        ControlMode::INPUT_VOLTAGE
    };
}

double MPCSerial::estimateA(const std::vector<double>& time, const std::vector<double>& omega, double omega_ss) {
    double sum = 0.0;
    int count = 0;
    for (std::size_t i = 0; i < omega.size(); i++) {
        double ratio = 1.0 - (omega[i] / omega_ss);
        if (ratio <= 0.0) {
            continue;
        }
        double a_i = -std::log(ratio) / time[i];
        if (std::isfinite(a_i)) {
            sum += a_i;
            count++;
        }
    }
    if (count == 0) {
        return 0.0;
    }
    return sum / count;
}

double MPCSerial::estimateB(double a, double omega_ss, double voltage) {
    return (a * omega_ss) / voltage;
}

void MPCSerial::runSingleIdentificationTest(int voltage) {
    std::vector<double> time;
    std::vector<double> omega;

    pros::delay(1000);
    int start = pros::millis();
    leftMotors.move_voltage(voltage);

    while (pros::millis() - start < 2000) {
        double t = (pros::millis() - start) / 1000.0;
        double w = leftMotors.get_actual_velocity(0) * 2.0 * M_PI / 60.0;
        time.push_back(t);
        omega.push_back(w);
        pros::delay(10);
    }

    leftMotors.move_voltage(0);
    double omega_ss = omega.back();
    double a = estimateA(time, omega, omega_ss);
    double b = estimateB(a, omega_ss, voltage / 12000.0);

    printf("\n");
    printf("Voltage: %d mV\n", voltage);
    printf("omega_ss: %f rad/s\n", omega_ss);
    printf("a: %f\n", a);
    printf("b: %f\n", b);
}

void MPCSerial::identifyMotorModel() {
    double sumA = 0.0;
    double sumB = 0.0;
    int count = 0;

    auto test = [&](int voltage) {
        std::vector<double> time;
        std::vector<double> omega;

        leftMotors.move_voltage(0);
        pros::delay(800);

        int start = pros::millis();
        leftMotors.move_voltage(voltage);

        while (pros::millis() - start < 2000) {
            double t = (pros::millis() - start) / 1000.0;
            double w = leftMotors.get_actual_velocity(0) * 2.0 * M_PI / 60.0;

            time.push_back(t);
            omega.push_back(w);

            pros::delay(10);
        }

        leftMotors.move_voltage(0);
        double omega_ss = omega.back();
        double a = estimateA(time, omega, omega_ss);
        double b = estimateB(a, omega_ss, voltage / 12000.0);

        printf("\nV: %d mV | a: %f | b: %f\n", voltage, a, b);

        sumA += a;
        sumB += b;
        count++;
    };
    test(5000);
    test(7500);
    test(10000);
    test(12000);

    double avgA = sumA / count;
    double avgB = sumB / count;

    printf("\n====================\n");
    printf("AVERAGE MODEL\n");
    printf("====================\n");
    printf("a: %f\n", avgA);
    printf("b: %f\n", avgB);
}