# Proposed `src/` and `include/` structure

This document describes the proposed target organization for Robot-Core and the role of its runtime tasks. The tree reflects the agreed direction, not a claim that every legacy file has already moved to its final location. Migrate existing code incrementally as modules are touched.

## Design goals

- Keep the PROS entry point small and easy to follow.
- Give each piece of robot behavior a clear home based on its responsibility.
- Use classes for stateful robot subsystems and pass shared subsystem
  references to autonomous routines.
- Keep project code under the `daydream` directory in both `src/` and
  `include/`, except for PROS-required root files such as `main.cpp` and
  `main.h`.
- Use `daydream` as the project namespace. Subsystem classes are named directly
  (for example, `daydream::Drivetrain`); the `subsystems/` directory organizes
  files and does not require a matching `daydream::subsystems` namespace.
- Keep archived or inactive work out of the directories PROS scans to compile
  the active application.

## Proposed tree

```text
Robot-Core/
|-- src/
|   |-- main.cpp                         PROS callbacks and object composition
|   `-- daydream/
|       |-- autonomous/
|       |   |-- autonomous.cpp            Autonomous movement behavior
|       |   |-- runner.cpp                 Autonomous sequencing entry point
|       |   `-- routines/autons.cpp        Named autonomous and skills routines
|       |-- motion/
|       |   |-- pid.cpp                    Reusable PID controller
|       |   |-- odometry.cpp               Continuous pose tracking
|       |   |-- control/
|       |   |   |-- mpcSerial.cpp           Serial MPC integration
|       |   |   |-- purePursuit.cpp         Pure-pursuit controller
|       |   |   |-- ramsete.cpp             Ramsete controller
|       |   |   `-- stanley.cpp              Stanley controller
|       |   |-- pathing/
|       |   |   |-- pathFollower.cpp        Path-following execution
|       |   |   |-- arclengthSplining.cpp   Spline construction and sampling
|       |   |   `-- paths.cpp               Path loading and preparation
|       |   `-- localization/
|       |       |-- mcl.cpp                 Monte Carlo localization
|       |       `-- rangeSensing.cpp         Range model and diagnostics
|       |-- subsystems/
|       |   |-- drivetrain.cpp              Drivetrain commands
|       |   |-- intake.cpp                  Intake commands
|       |   |-- logging.cpp                 Robot-wide logging interface
|       |   `-- pneumatics.cpp              Pneumatic actuator commands
|       |-- utilities/display.cpp           Driver display and selection UI
|       `-- utils/serialProtocol.cpp        Serial packet transport
|-- include/
|   |-- api.h                               PROS API umbrella header
|   |-- main.h                              PROS lifecycle declarations
|   |-- pros/                               PROS SDK headers
|   |-- liblvgl/                            LVGL vendor headers
|   |-- gif-pros/                           GIF PROS vendor headers
|   `-- daydream/
|       |-- autonomous/
|       |   |-- autonomous.hpp
|       |   |-- runner.hpp                  Shared-object autonomous entry point
|       |   `-- routines/autons.hpp
|       |-- config/
|       |   |-- constants.hpp               Hardware ports and tuning values
|       |   `-- hardware.hpp                Configured PROS device declarations
|       |-- motion/
|       |   |-- pid.hpp
|       |   |-- odometry.hpp
|       |   |-- control/
|       |   |   |-- motionController.hpp     Common controller interface
|       |   |   |-- mpcSerial.hpp
|       |   |   |-- purePursuit.hpp
|       |   |   |-- ramsete.hpp
|       |   |   `-- stanley.hpp
|       |   |-- pathing/
|       |   |   |-- pathFollower.hpp
|       |   |   |-- arclengthSplining.hpp
|       |   |   `-- paths.hpp
|       |   `-- localization/
|       |       |-- mcl.hpp
|       |       `-- rangeSensing.hpp
|       |-- subsystems/
|       |   |-- drivetrain.hpp              `daydream::Drivetrain`
|       |   |-- intake.hpp                  `daydream::Intake`
|       |   |-- logging.hpp                 `daydream::Logging`
|       |   `-- pneumatics.hpp              `daydream::Pneumatics`
|       |-- utilities/
|       |   |-- display.hpp
|       |   `-- math.hpp                    Header-only shared math helpers
|       `-- utils/
|           |-- fieldLogger.hpp              Path/controller data logging
|           |-- rng.hpp                      Random-number helper
|           |-- sd_card_logging.hpp          SD-card logging support
|           `-- serialProtocol.hpp           Serial packet declarations
`-- archives/
    |-- legacy_path_follower_main.cpp
    |-- legacy_example_main.cpp
    |-- legacy_pneumatics.cpp
    |-- legacy_pneumatics.hpp
    `-- perception/                          Inactive perception work
        |-- include/
        |   |-- object_handler.hpp
        |   `-- vision_ball_retrieval.hpp
        `-- src/
            |-- object_handler.cpp
            `-- vision_ball_retrieval.cpp
```

This is the target proposal. Some existing files still use older paths or names, including the current MCL, helper, and path-follower files; those are migration items. New files should follow this mirroring rule: a public header in
`include/daydream/<area>/` should have its implementation in the corresponding
`src/daydream/<area>/` directory. Header-only helpers are an exception when
their implementation is intentionally defined in the header.

## Runtime structure and PROS tasks

### `main.cpp` is the composition root

`main.cpp` is where the application connects hardware configuration to robot
behavior. It defines the PROS lifecycle callbacks and owns the long-lived
instances of the project subsystem classes. It should stay focused on startup,
mode dispatch, and wiring shared objects together; detailed movement algorithms
and mechanism behavior belong in their respective modules.

The current scaffold creates one instance each of `daydream::Drivetrain`,
`daydream::Intake`, `daydream::Pneumatics`, and `daydream::Logging`. The
drivetrain and pneumatic subsystem wrappers receive references to the existing
PROS device objects. Those declarations currently live in
`config/subsystems.hpp`; the proposed name for that hardware configuration file
is `config/hardware.hpp`. This avoids creating second PROS objects for the same
physical ports. Function-local statics keep the subsystem objects alive across
callback invocations while ensuring their construction is deferred until first
use.

### Competition modes already run as PROS-managed tasks

PROS invokes `autonomous()` and `opcontrol()` as competition-mode callbacks.
The project should treat those as the top-level autonomous and driver-control
execution contexts. It generally should not create another task just to run a
mechanism or a complete autonomous routine. The mode callback can call
subsystem methods and routines directly, with appropriate periodic delays in
control loops.

The operator-control loop in `opcontrol()` currently updates the drivetrain,
intake, and pneumatics, then yields for 20 ms. This establishes a simple
periodic control loop. As the project grows, the loop can call a single
per-cycle update on each subsystem; a separate task should be introduced only
when work truly needs to run concurrently or has a distinct timing requirement.

### Background tasks are for continuous shared services

Odometry is a continuous service needed during both autonomous and driver
control. The scaffold starts `Odometry::odomTask` once from initialization and
retains its `pros::Task` object for the lifetime of the application. Autonomous
and operator control read the same odometry service rather than each starting
their own updater.

When adding another background task, document its owner, lifetime, update rate,
and shared data. Protect data that can be read or written from more than one
task with an appropriate synchronization mechanism. Avoid duplicate workers,
and make sure a task does not compete with the active mode for actuator
commands. A mechanism task is only justified if that mechanism must perform
independent concurrent work; otherwise its update belongs in the active mode's
periodic loop.

### Autonomous and operator control share subsystem objects

The autonomous runner accepts references to the same drivetrain, intake,
pneumatics, and odometry objects that the application uses elsewhere. A
routine can therefore command the robot and read its pose without constructing
hardware objects or depending on global motor groups directly. Later systems,
such as localization or an odometry-based navigation service, can be supplied
to autonomous in the same way when their interfaces are ready.

Competition modes are mutually selected by PROS, so only the active mode should
issue normal actuator commands. The long-lived subsystem objects may persist
between modes; the mode callback determines who is currently using them.

## Folder responsibilities

### `subsystems/`

Contains stateful, robot-facing units of behavior that group hardware access
with meaningful commands. Examples include drivetrain, intake, pneumatics, and
robot-wide logging. Each subsystem should expose a small public interface and
hide low-level details behind it. Pass hardware dependencies into a subsystem
when practical so the class does not create another object for an already
configured device.

Classes live directly in `namespace daydream`, for example:

```cpp
daydream::Drivetrain drivetrain(leftMotors, rightMotors);
```

This keeps call sites readable and avoids repeating the folder name as a nested
namespace. `subsystems/` is an organizational boundary, not a required part of
the C++ type name.

### `autonomous/`

Contains autonomous motion behavior, a runner that sequences selected
autonomous actions, and the named routines used by the team. The runner should
receive references to shared services and subsystems. Routines should describe
what the robot does; they should not create hardware objects, start the
operator-control loop, or create duplicate odometry workers.

### `motion/`

Contains robot motion and pose-related algorithms. Keep PID directly in `motion/` because it is a reusable motion-control primitive, rather than giving it a separate top-level subsystem folder. `control/` contains algorithms that turn
pose/path targets into drive commands. `pathing/` owns path representation,
spline generation, path loading, and following a path. Odometry belongs directly
under `motion/` because it estimates the robot's motion and pose.

Localization that depends on range sensors and a particle filter belongs under `motion/localization/`. This keeps it with pose-estimation and navigation concerns that consume its results. Inactive perception experiments remain under `archives/`.

### `utilities/` and `utils/`

Use `utilities/` for shared tools such as the display and math helpers. Keep common math in `utilities/math.hpp`; a separate `math.cpp` is unnecessary while these helpers are small and header-only. Move sensor- and odometry-specific helpers next to localization or odometry instead of keeping them in a generic catch-all file. Use `utils/` for lower-level reusable pieces such as random-number generation, serial transport, and logging support.

Logging has two related scopes. Robot-wide event or diagnostic logging belongs
behind the `Logging` subsystem interface. Path-following or controller data
that is specific to trajectory analysis belongs with the pathing/controller
code (the current `FieldLogger` helper is in `utils/`). Keep the call-site
interface straightforward and place implementation according to who owns the
data and lifecycle.

### `config/`

Contains robot-specific configuration: physical ports, dimensions, tuning
constants, and the existing PROS device declarations. `constants.hpp` is the
proposed shared constants filename; it should not grow a robot-prefixed name.
`hardware.hpp` is the proposed home for configured PROS device objects. As the
hardware setup evolves, keep the boundary clear between constant configuration
and subsystem behavior.

### `archives/`

Contains inactive or historical project code, including perception work that
is not part of the active build. Keeping it outside `src/` and `include/`
prevents old files from being picked up by recursive source discovery and makes
the active code easier to navigate. Archive files are references, not part of
the supported application interface.

## Naming and dependency guidelines

- Mirror paths between `include/daydream/` and `src/daydream/`.
- Use descriptive filenames and matching header/source stems, such as
  `motion/pathing/paths.hpp` and `motion/pathing/paths.cpp`.
- Prefer `.hpp` for C++ project headers. Preserve `.h` where an existing
  integration or build convention requires it until it can be changed safely.
- Keep hardware object construction in configuration/application wiring;
  subsystem classes should receive references when they wrap existing devices.
- Keep ownership visible. Long-lived services belong to the application
  composition layer; autonomous routines and controllers borrow references.
- Include the narrowest project header needed rather than relying on an
  unrelated umbrella include.
- Keep algorithms independent of the PROS competition callbacks where
  practical. This makes modules easier for team members to understand and
  allows shared services to be reused from both autonomous and driver control.
- Do not expand the skeleton into a full rewrite as part of routine feature
  work. Migrate existing algorithms behind the interfaces incrementally and
  update this proposal when the intended architecture changes.

## Scope of the current scaffold

The current scaffold establishes the application boundary and shared-object
interfaces. It does not yet migrate all legacy motion algorithms to use
`daydream::Drivetrain`, fully implement intake behavior, select among
autonomous routines, or define the final navigation/localization interface.
Those are follow-on implementation tasks. New code should build on the shared
interfaces rather than introducing parallel hardware ownership or additional
mode-level tasks without a concrete concurrency need.

