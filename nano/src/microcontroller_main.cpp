#define _USE_MATH_DEFINES

#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include "mpc.hpp"
#include "serialProtocol.hpp"

//serial packets exchanged with the vex brain (layout must match the brain's MPCSerial)
#pragma pack(push, 1)
template<std::size_t F>
struct MPCUpdatePacket {
    float pose_x;
    float pose_y;
    float pose_theta;
    float omega_L;
    float omega_R;
    float V_battery;
    float I_total;
    //F+1 reference poses (x, y, theta)
    float z_desired[(F + 1) * 3];
};
#pragma pack(pop)

#pragma pack(push, 1)
struct MPCControlPacket {
    float V_left;
    float V_right;
};
#pragma pack(pop)

//unpack z_desired from float array
template<std::size_t V, std::size_t F>
auto unpackZDesired(const float* z_raw) {
    Eigen::Matrix<double, MPCController<V, F>::r_states * (F + 1), 1> z;
    for (std::size_t i = 0; i < F + 1; i++) {
        const std::size_t base = i * 3;
        z(base + 0) = static_cast<double>(z_raw[base + 0]);
        z(base + 1) = static_cast<double>(z_raw[base + 1]);
        z(base + 2) = static_cast<double>(z_raw[base + 2]);
    }
    return z;
}

//function to recieve update packet: compute voltages: send control packet
template<std::size_t V, std::size_t F>
void MPCControl(SerialProtocol& serial, MPCController<V, F>& mpc) {
    //wait for state packet from vex brain
    std::optional<MPCUpdatePacket<F>> packet_opt = serial.receive<MPCUpdatePacket<F>>(SerialProtocol::PacketType::MPC_UPDATE);
    //timeout or bad packet
    if(!packet_opt.has_value()) {
        return;
    }

    const MPCUpdatePacket<F>& packet = *packet_opt;

    //unpack pose
    Pose pose{
        packet.pose_x,
        packet.pose_y,
        packet.pose_theta
    };

    //unpack reference trajectory
    auto z_desired = unpackZDesired<V, F>(packet.z_desired);

    //initialize MPCControlPacket with default failsafe of 0V
    MPCControlPacket response{0.0f, 0.0f};
    //run MPC optimization with exception safety
    try {
        WheelVelocities out = mpc.compute(pose, z_desired, packet.omega_L, packet.omega_R, packet.V_battery, packet.I_total);
        //pack optimal voltages on success
        response.V_left = static_cast<float>(out.left);
        response.V_right = static_cast<float>(out.right);
    }
    catch (const std::exception& e){
        //MPC failed: log error and send zero volts
        std::cerr << "MPC ERROR: " << e.what() << ", sending zero volts\n";
    }
    //send optimal voltages
    serial.send(SerialProtocol::PacketType::MPC_CONTROL, response);
    std::cerr << "sent V_L=" << response.V_left << " V_R=" << response.V_right<<  std::endl;
}

int main() {
    //initiailizes prediction and control horizon (compile-time)
    using V = std::integral_constant<std::size_t, 15>;
    using F = std::integral_constant<std::size_t, 15>;
    // create MPC with parameters
   typename MPCController<V::value, F::value>::Params params(
        1.625, //wheel radius (in)
        10.5, //track width (in)
        16.7211, //motor time-constant
        71.3105, //motor gain constant
        0.02, //sampling period (s)
        200.0, //output weight for x
        200.0, //output weight for y
        60.0, //output weight for theta
        0.1, //input penalty q_u
        2.0, //multiplier for first Q_i
        1.0, //multiplier for last Q_i
        20.0, //multiplier for last P_i
        12.0, //voltage max
        0.083, //internal resistance of battery
        8.0, //max positive change in voltage between steps
        -8.0, //max negative change in voltage between steps
        140, //max allowed x position on field
        140, //max allowed y position on field
        100, //max allowed rad/s
        0.0, //starting Voltage left
        0.0 //starting Voltage right
    );
    //initialize the MPC solver and  reset() the internal state
    MPCController<V::value, F::value> mpc(params);
    mpc.reset();

    //open serial to communicate for vex brain
    int fd = SerialProtocol::setUpMicrocontrollerSerial("/dev/ttyACM1", B115200);
    if (fd < 0) {
        std::cerr << "Failed to open serial\n";
        return 1;
    }

    // disable buffering (this might not be needed anymore because it is done in setUp...)
    setbuf(stdin,  NULL);
    setbuf(stdout, NULL);
    setvbuf(stdin,  NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);

    //initialize 8 kB binary link
    SerialProtocol serial(8192, SerialProtocol::Mode::BINARY);

    std::cerr << "MPC micro waiting for VEX...\n";

    //wait for vex brain to boot
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    std::cerr << "Ready\n";
    //background heartbeat which send a null wakeup byte every 500 ms so the brain knows the coprocessor is alive
	std::thread wakeupTask([&serial]() {
    while (true) {
        serial.sendWakeup(std::string(1, '\0'));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
});

//heartbeat runs independently for the life of the process
wakeupTask.detach();
    //50 Hz control loop
    while (true) {
        //recieve update packet: compute voltages: send control packet
        MPCControl(serial, mpc);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

