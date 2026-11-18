# Serial protocol integration status

## Current status

The Brain and Nano serial link implementations are present, but the link is **not started by either application**. The Brain runtime is an unfinished scaffold. The Nano `MPCController` executable currently prints `Serial link is not configured; runtime startup is disabled.` and exits with status 1. The MPC library is still built, but the executable does not run the solver or control the robot. The former MPC-specific serial adapter and Nano entry point are retained under `archive/`.

Hardware communication, pin selection, timing, and recovery have not been verified on the robot. The examples below show how to use the existing API when integrating it; they are not active application code or a confirmed wiring configuration.

## Code and protocol

- [`shared/include/protocol/serial.hpp`](../shared/include/protocol/serial.hpp) declares `SerialLink` and `packets::buildLinkConfig()`.
- [`shared/include/protocol/packets.hpp`](../shared/include/protocol/packets.hpp) defines the packet layouts, enums, status and fault flags, and size checks.
- [`shared/include/protocol/serialProtocol.hpp`](../shared/include/protocol/serialProtocol.hpp) declares the internal framing and transport interface. Shared framing and parsing are implemented in [`shared/src/protocol/serialProtocol.cpp`](../shared/src/protocol/serialProtocol.cpp); the Brain and Nano platform code is under each project's `src/link/` directory.
- [`shared/include/config/linkConstants.hpp`](../shared/include/config/linkConstants.hpp) supplies the motor, tracking wheel, and ADI assignments used by the default configuration. Rebuild both sides with the same configuration when changing these assignments or the configuration builder.

Frames are little-endian: `55 AA | type u16 | payload size u16 | payload | CRC-16 u16`. The CRC covers type, size, and payload. The protocol is half-duplex: the Brain sends `STATE` and `LOG`; the Nano replies to a state with `COMMAND` or, when configurations differ, `CONFIG`. The default config uses a 10 ms state period and a 100 ms autonomous watchdog. These are code settings, not hardware-verified timing guarantees.

The current build passes `__v5_brain__` for the Brain and `__jetson_nano__` for the Nano. Both compile the shared framing implementation with their own platform implementation. `SerialProtocol` is internal to `SerialLink`; use `SerialLink` for application integration.

## Configuration and ownership

`packets::buildLinkConfig()` builds the same configuration payload on each side and derives its nonzero ID from the payload. It describes left and right drive groups, parallel and perpendicular tracking sensors, and the Brain ADI outputs in `linkConstants.hpp`. The Brain attempts to apply its built-in configuration to the devices supplied by the owner; the Nano sends its config when it sees a different ID. A matching ID is not proof of a working physical link.

The Brain link defaults to Nano control enabled. `setNanoControl(false)` prevents the link from applying motor and ADI commands; while the robot is enabled it also suppresses state frames, although queued logs can still be sent. While disabled, state transmission continues. `setNanoControl(true)` allows link commands to affect the configured motors and ADI outputs; coordinate this with existing Brain subsystems so they do not command the same devices at the same time. One-shot requests are handled separately from that output gate. The watchdog code brakes groups after missing commands in autonomous; verify this behavior with the actual robot before relying on it.

Keep each `SerialLink` object alive for the duration of its tasks or threads. Construct it once and call `start()` once. Neither constructor opens the port. The Nano destructor stops and joins its threads; the Brain destructor removes its tasks.

## Brain API example

This example illustrates USB only. For RS-485, use the **confirmed** Brain smart port, baud rate, and `SerialProtocol::Transport::RS485` instead. The smart port must not conflict with a configured device. USB uses the Brain's stdin/stdout transport; ordinary output on that stream can interfere with binary frames.

```cpp
#include "api.h"
#include "brain/communication/nanoLink.hpp"
#include "protocol/serial.hpp"
#include <array>
#include <vector>

// The caller owns the controller and every device, and keeps them alive until
// after both link objects are stopped and destroyed. Outputs are indexed by
// protocol ADI slot; unused slots may be null. Groups and wheels are in slot order.
void startNanoLink(SerialLink& link) {
    // Retain Brain control until the application explicitly hands it over.
    link.setNanoControl(false);
    if (!link.start()) {
        pros::lcd::print(3, "Nano link failed to open");
        return;
    }
    link.log(0, "Brain link started");
}

// Once the caller has created its actual devices:
// brain::NanoLink devices(controller, groups, wheels, outputs);
// SerialLink link(devices, packets::buildLinkConfig(), 0, 0,
//                 SerialProtocol::Transport::USB);
// startNanoLink(link);
// link.setNanoControl(true);  // Hand output control to the Nano.
```

`start()` validates the supplied group and wheel bindings and attempts to apply the boot configuration to those existing devices; it does not create them. Calling `setNanoControl(false)` does not prevent configuration. A `true` return means the bindings and port passed startup checks and task creation was attempted; it does **not** guarantee that the configuration was accepted, the tasks started successfully, or a Nano is connected. A rejected boot configuration is reported in state while the link waits for a usable configuration. `transmitErrors()` counts failed Brain frame writes.

## Outstanding Robot integration and hardware verification

- `Robot` must own the controller and devices, provide the corresponding non-owning bindings, and ensure they outlive `NanoLink` and `SerialLink`. A higher-level mutex or task-ownership policy must coordinate every user of each device; `NanoLink`'s private mutex does not synchronize other components.
- The shared protocol permits configuration changes to device ports. The current fixed bindings reject a different group, rotation sensor, or ADI port; supporting changes requires the Robot device owner to replace or rebind devices safely and coordinate existing references.
- Command reception time is sampled when software obtains a parsed frame, before waiting for the link lock or applying outputs. Bytes already buffered or delayed scheduling can make that timestamp later than the physical end of reception; verify time synchronization on hardware.
- Verify ADI expander reconnect and output configuration, USB task removal and restart during blocked I/O, write failures, motor braking, and serial recovery on the actual Brain and Nano. No hardware behavior is established by compilation alone.

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
            // Example: sample->imeRad[packets::LEFT_DRIVE_SLOT], after checking size.
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}
```

The current Nano code accepts numeric baud rates 115200, 230400, 460800, and 921600. `start()` returns `false` for an unsupported baud, invalid configuration, or failed port/GPIO setup. It does not mean the Brain is connected; use `linkUp()` and `latestSample()` after startup. `onSample()` registers a callback on the reader thread, so keep it short.

`Sample` contains reconstructed group and tracking-wheel positions and steps in radians, group speed in rad/s at the motor output shaft, battery millivolts, controller data, status, error, and fault flags. Group and wheel vectors follow the slot constants in `serial.hpp`; check vector sizes before indexing. Convert motor-shaft angles and speeds using the robot's external gearing when wheel values are needed.

To issue commands after the application has established appropriate control ownership:

```cpp
link.setGroup(packets::LEFT_DRIVE_SLOT, packets::GroupMode::STOP, 0);
// For a voltage command, value is millivolts, e.g. GroupMode::VOLTAGE, 1000.
// link.setAdi(0, true);  // ADI slot 0 is Brain port A, if enabled in the config.
// auto id = link.request(packets::Request::GROUP_TARE, packets::LEFT_DRIVE_SLOT);
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
- The Brain's `start()` reports rejected configuration in state while continuing to run after valid bindings and port setup. Task-start failure reporting remains unresolved. Nano real-time scheduling failures are also ignored.
- Verify retry, shutdown, and safe output behavior under errors and lost communication before relying on the interface for robot control.
