#include "mpc_serial.hpp"
#include "subsystems.hpp"
#include <cmath>
#include <algorithm>

MPCSerial::Params::Params(
    double frequency
)
    : h(frequency) {}

MPCSerial::MPCSerial(const Params& params)
    : m_params(params), serial(1024, SerialProtocol::Mode::BINARY)
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

        double t =
            (pros::millis() - start) / 1000.0;

        double w = leftMotors.get_actual_velocity(0)* 2.0 * M_PI / 60.0;

        time.push_back(t);
        omega.push_back(w);

        pros::delay(10);
    }

    leftMotors.move_voltage(0);

    double omega_ss = omega.back();

    double a =
        estimateA(time, omega, omega_ss);

    double b =
        estimateB(a, omega_ss, voltage / 12000.0);

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
            double w = leftMotors.get_actual_velocity(0)* 2.0 * M_PI / 60.0;

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
