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
        10.63, //track width (in)
        10.4151, //motor time-constant
        730.8520, //motor gain constant
        0.02, //sampling period (s)
        2.0, //output weight for x
        15.0, //output weight for y
        100.0, //output weight for theta
        0.01, //input penalty
        0.1, //multiplier for first Q_i
        5.0, //multiplier for last Q_i
        10.0, //multiplier for last P_i
        12.0, //voltage max
        0.083, //internal resistance of battery
        4.0, //max positive change in voltage between steps
        -4.0, //max negative change in voltage between steps
        70.2, //max allowed x position on field
        70.2, //max allowed y position on field
        47.13, //max allowed rad/s
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

    //50 Hz control loop
    while (true) {
        //recieve update packet: compute voltages: send control packet
        MPCController<V::value, F::value>::MPCControl(serial, mpc);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}
