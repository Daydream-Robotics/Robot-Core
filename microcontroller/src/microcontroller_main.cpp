#define _USE_MATH_DEFINES

#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include "mpc.hpp"
#include "serialProtocol.hpp"

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
        0.1, //input penalty
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
	std::thread wakeupTask([&serial]() {
    while (true) {
        serial.sendWakeup(std::string(1, '\0'));

        std::this_thread::sleep_for(
            std::chrono::milliseconds(500)
        );
    }
});

wakeupTask.detach();
    //50 Hz control loop
    while (true) {
        //recieve update packet: compute voltages: send control packet
        MPCController<V::value, F::value>::MPCControl(serial, mpc);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

