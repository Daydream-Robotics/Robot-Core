# Serial protocol integration status

## Current status

The Brain and Nano serial link implementations are present and both projects build. The Brain application starts the link in `initialize()` (`nanoLink` in `v5-brain/include/config/subsystems.hpp`, transport and smart port from the `[NANO]` section of `shared/include/config/constants.h`), keeps control in driver with `setNanoControl(false)`, and hands the drive to the Nano with `setNanoControl(true)` in `autonomous()`. The Nano `MPCController` executable starts it (device, transport, baud, DE GPIO and path file come from the `[NANO]` section of `shared/include/config/constants.h`), and during an enabled autonomous period runs the MPC on each path and sends drive voltages in `COMMAND`; outside autonomous it sends `STOP`. It has no pose source yet: `latestPose()` in `jetson-nano/src/main.cpp` returns nothing until the EKF is connected, so the drive holds 0 V. See [`MPC_DOCS.md`](MPC_DOCS.md). The former MPC-specific serial adapter and Nano entry point are retained under `archive/`.

Hardware communication, pin selection, timing, and recovery have not been verified on the robot. The examples below show how to use the existing API when integrating it; they are not active application code or a confirmed wiring configuration.

## Code and protocol

- [`shared/include/protocol/serial.hpp`](../shared/include/protocol/serial.hpp) declares `SerialLink` and `packets::buildLinkConfig()`.
- [`shared/include/protocol/packets.hpp`](../shared/include/protocol/packets.hpp) defines the packet layouts, enums, status and fault flags, and size checks.
- [`shared/include/protocol/serialProtocol.hpp`](../shared/include/protocol/serialProtocol.hpp) declares the internal framing and transport interface. Shared framing and parsing are implemented in [`shared/src/protocol/serialProtocol.cpp`](../shared/src/protocol/serialProtocol.cpp); the Brain and Nano platform code is under each project's `src/link/` directory.
- [`shared/include/config/linkConstants.hpp`](../shared/include/config/linkConstants.hpp) supplies the motor, tracking wheel, and ADI assignments used by the default configuration. Rebuild both sides with the same configuration when changing these assignments or the configuration builder.

Frames are little-endian: `55 AA | type u16 | payload size u16 | payload | CRC-16 u16`. The CRC covers type, size, and payload. The protocol is half-duplex: the Brain sends `STATE` and `LOG`; the Nano replies to a state with `COMMAND` or, when configurations differ, `CONFIG`. The default config uses a 10 ms state period and a 100 ms autonomous watchdog. These are code settings, not hardware-verified timing guarantees.

The current build passes `__v5_brain__` for the Brain and `__jetson_nano__` for the Nano. Both compile the shared framing implementation with their own platform implementation. `SerialProtocol` is internal to `SerialLink`; use `SerialLink` for application integration.

## Configuration and ownership

`packets::buildLinkConfig()` builds the same configuration payload on each side and derives its nonzero ID from the payload. It configures the drive groups of the robot's drivetrain, parallel and perpendicular tracking sensors, and the Brain ADI outputs described in `linkConstants.hpp`. The drivetrain is `RobotDrive` in `shared/include/config/robotDrive.hpp` (`kinematics::DiffDrive`, `kinematics::XDrive` or `kinematics::HDrive`). Drive groups take the first slots in the drivetrain's group order: diff L, R; X FL, FR, BL, BR; H L, R, strafe. `packets::driveSlot(group)` gives a group's slot and `packets::DRIVE_GROUPS` the count. Changing `RobotDrive` changes the config and its ID, so both the Brain and the Nano must be rebuilt with the same setting. The X and H drive ports in `linkConstants.hpp` are placeholders until those robots are built, and their signs must make positive voltage drive each group its positive way (forward on every X wheel and H side, left on the H strafe wheel). The Brain applies its built-in configuration on startup; the Nano sends its config when it sees a different ID. A matching ID is not proof of a working physical link.

The Brain link defaults to Nano control enabled. `setNanoControl(false)` prevents the link from applying motor and ADI commands; while the robot is enabled it also suppresses state frames, although queued logs can still be sent. While disabled, state transmission continues. `setNanoControl(true)` allows link commands to affect the configured motors and ADI outputs; coordinate this with existing Brain subsystems so they do not command the same devices at the same time. One-shot requests are handled separately from that output gate. The watchdog code brakes groups after missing commands in autonomous; verify this behavior with the actual robot before relying on it.

Keep each `SerialLink` object alive for the duration of its tasks or threads. Construct it once and call `start()` once. Neither constructor opens the port. The Nano destructor stops and joins its threads; the Brain destructor removes its tasks.

## Brain API example

This example illustrates USB only. For RS-485, construct the link with the **confirmed** Brain smart port, baud rate, and `SerialProtocol::Transport::RS485` instead. The smart port must not conflict with a configured device. USB uses the Brain's stdin/stdout transport; ordinary output on that stream can interfere with binary frames.

```cpp
#include "api.h"
#include "protocol/serial.hpp"

// Keep this object alive while its tasks run. USB ignores port and baud.
SerialLink nanoLink(0, 0, packets::buildLinkConfig(),
                    SerialProtocol::Transport::USB);

void startNanoLink() {
    // Retain Brain control until the application explicitly hands it over.
    nanoLink.setNanoControl(false);
    if (!nanoLink.start()) {
        pros::lcd::print(3, "Nano link failed to open");
        return;
    }
    nanoLink.log(0, "Brain link started");
}

// At an application-controlled handoff:
// nanoLink.setNanoControl(true);   // Nano may command configured outputs.
// nanoLink.setNanoControl(false);  // Brain controls configured outputs.
```

`start()` applies the configuration, which can create and configure motor, rotation, and ADI devices. Calling `setNanoControl(false)` does not prevent that configuration step. A `true` return currently means port setup succeeded and task creation was attempted; it does **not** guarantee the Brain configuration was accepted, the tasks started successfully, or that a Nano is connected. See the error-handling follow-up below.

## Nano API example

Pass a device path discovered on the actual system. The following example selects USB; a typical `/dev/ttyACM*` name is only an example and can vary. For RS-485, pass the confirmed UART device, matching baud, `SerialProtocol::Transport::RS485`, and either a confirmed **sysfs/kernel GPIO number** or `-1` if the transceiver handles direction automatically.

```cpp
#include <chrono>
#include <iostream>
#include <thread>
#include "protocol/serial.hpp"

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: link-example /dev/ttyACM...\n";
        return 1;
    }

    SerialLink link(argv[1], 115200, -1, packets::buildLinkConfig(),
                    SerialProtocol::Transport::USB);
    if (!link.start()) {
        std::cerr << "Could not start Brain link\n";
        return 1;
    }

    for (;;) {
        const auto sample = link.latestSample();
        if (sample && link.linkUp()) {
            // Check sample->status, sample->lastError, and device flags before use.
            // Example: sample->imeRad[packets::driveSlot(0)], after checking size.
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
```

The current Nano code accepts numeric baud rates 115200, 230400, 460800, and 921600. `start()` returns `false` for an unsupported baud, invalid configuration, or failed port/GPIO setup. It does not mean the Brain is connected; use `linkUp()` and `latestSample()` after startup. `onSample()` registers a callback on the reader thread, so keep it short.

`Sample` contains reconstructed group and tracking-wheel positions and steps in radians, group speed in rad/s at the motor output shaft, battery millivolts, controller data, status, error, and fault flags. Group and wheel vectors follow the slot constants in `serial.hpp`; check vector sizes before indexing. Convert motor-shaft angles and speeds using the robot's external gearing when wheel values are needed (`driveGearRatio(group)` in `robotDrive.hpp` for the drive groups).

To issue commands after the application has established appropriate control ownership:

```cpp
link.setGroup(packets::driveSlot(0), packets::GroupMode::STOP, 0);
// For a voltage command, value is millivolts, e.g. GroupMode::VOLTAGE, 1000.
// link.setAdi(0, true);  // ADI slot 0 is Brain port A, if enabled in the config.
// auto id = link.request(packets::Request::GROUP_TARE, packets::driveSlot(0));
// bool acknowledged = link.requestDone(id);
```

Group commands persist until replaced; valid modes include stop, voltage in mV, velocity in motor-output rpm, and position in degrees with maximum rpm in `aux`. ADI states persist after `setAdi()`. A new one-shot request replaces any request still in flight; an acknowledgment means its ID was echoed, so also inspect the reported error. A stale sample can remain available after a disconnect: check `linkUp()` before acting on it.

## Hardware configuration still to confirm

- Confirm the Jetson model, carrier, software release, UART device and pinmux, Brain smart port, matching baud, transceiver wiring and signal levels. `/dev/ttyTHS1` is only an example in the API, not a verified assignment for this robot.
- Confirm whether the transceiver is auto-direction. `deGpio = -1` means **do not drive a DE GPIO**; it is not a startup error or a substitute for a required direction pin. Otherwise provide the Linux sysfs GPIO number, which is not necessarily the 40-pin header number. The implementation assumes one output drives DE and `/RE` together: low for receive and high for transmit.
- Confirm permissions for the serial device and, when used, GPIO export/direction/value. GPIO uses the legacy sysfs interface and must be available on the deployed system. Nano threads attempt `SCHED_FIFO` priority; failure currently leaves normal scheduling without reporting it.
- With hardware available, verify bidirectional frames, direction transitions and release timing, configuration agreement, control handoff, watchdog behavior, disconnect/reconnect, and startup failure behavior. The implementation waits 200 microseconds before the Nano reply and skips replies starting after 2 ms; actual timing remains unverified.

## Error-handling follow-up

These are observations in the current code, not fixes included in this document:

- Nano GPIO setup retries setting output-low for up to one second and returns `false` if direction or value setup fails. If failure occurs after the UART is opened, the UART remains open until the protocol object is destroyed; review cleanup and retry behavior.
- `setDE()` ignores the result of GPIO `pwrite()`, and the UART writer ignores the result of `tcdrain()`. Report or propagate these failures and define how to recover if direction cannot be asserted or released.
- The writer's `send()` result is not surfaced by the Nano link loop. Read errors are treated like no data, so transport failure and timeout are not clearly distinguished. Define diagnostics and recovery for failed sends, reads, and disconnects.
- The Brain's `start()` records configuration errors but still starts its tasks and returns `true` after successful port setup. Decide how callers should receive configuration and task-start failures. Nano real-time scheduling failures are also ignored.
- Verify retry, shutdown, and safe output behavior under errors and lost communication before relying on the interface for robot control.
