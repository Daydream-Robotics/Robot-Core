#define _USE_MATH_DEFINES

#include <unistd.h>
#include <iostream>
#include <chrono>
#include <thread>
#include "mpc.hpp"
#include "daydream/utils/serial/serial.hpp"

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

    // link to the brain over its USB port. -1 = no DE pin
    SerialLink link("/dev/ttyACM1", 115200, -1, packets::buildLinkConfig(), SerialProtocol::Transport::USB);
    // link to the brain over RS-485 on the Jetson's UART. -1 = auto-direction transceiver, otherwise the DE gpio number
    // SerialLink link("/dev/ttyTHS1", NANO_LINK_BAUD, -1, packets::buildLinkConfig());

    // open the port and start the reader and writer threads
    if (!link.start()) {
        std::cerr << "failed to open the brain link\n";
        return 1;
    }
    std::cerr << "waiting for the brain...\n";

    // stateSeq of the last Sample handled
    uint16_t lastSeq = 0;
    bool haveSample = false;

    // control loop: one pass per new Sample
    while (true) {
        std::optional<SerialLink::Sample> sample = link.latestSample();
        // no Sample yet, link down, or nothing new since the last pass
        if (!sample || !link.linkUp() || (haveSample && sample->stateSeq == lastSeq)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        haveSample = true;
        lastSeq = sample->stateSeq;

        // enabled autonomous: the Nano is in control
        bool auton = (sample->status & packets::status::AUTON) && !(sample->status & packets::status::DISABLED);

        // TODO: in auton, run odometry and the MPC here and send its voltages (mV) with
        // link.setGroup(packets::LEFT_DRIVE_SLOT, packets::GroupMode::VOLTAGE, ...)
        // until then the drive is held stopped in every mode
        (void)auton;
        link.setGroup(packets::LEFT_DRIVE_SLOT, packets::GroupMode::STOP, 0);
        link.setGroup(packets::RIGHT_DRIVE_SLOT, packets::GroupMode::STOP, 0);
    }
}

