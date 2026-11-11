# Integration and odometry updates merge

This incremental merge combines `dev/integration` at `beb561453bf6` and
`dev/odomUpdates` at `2fe07c662838`. The goal is to place the incoming work
in the Nano-oriented project structure while preserving its behavior. This
merge does not connect the robot or require the project to build.

## Placement and conflict decisions

- The incoming odometry, MCL and range-sensing headers and sources are under
  `jetson-nano/include/localization/` and `jetson-nano/src/localization/`.
  The incoming MCL header is included even though it was not changed by the PR.
- The incoming PID, pure pursuit, Ramsete and Stanley files are under the Nano
  motion-control directories, including their matching headers. The incoming
  random generator header is under `jetson-nano/include/utils/`.
- The incoming helper header and source are preserved as `odomHelpers.hpp` and
  `odomHelpers.cpp` beside integration's existing Nano helpers. Their original
  includes and Brain dependencies have not been connected or adapted.
- Integration's Nano autonomous header and source remain at the active paths,
  since the Nano entry point and `MPCController` target depend on them. The
  incoming `Autonomous` header and source are preserved in
  `archive/merge-odom-updates/odomUpdates/`. Incoming odometry takes the active
  Nano odometry header path, and integration's displaced Nano odometry header
  is preserved in `archive/merge-odom-updates/integration/`.
- Integration's Nano spline, JerryIO path loader, serial implementation, MPC
  and MPC reference remain in place. The Nano spline and incoming spline both
  use compass headings, but their curvature conventions differ. Integration's
  path loader keeps JerryIO paths in the field frame; the incoming loader also
  contains a different normalization approach. The incoming spline and path
  files are preserved in `archive/merge-odom-updates/odomUpdates/` for review.
- The incoming Brain-side `MPCSerial` source and header are archived there as
  well. Its compass-to-math conversion is not inserted into integration's Nano
  MPC, whose model already uses compass-style headings.
- `shared/include/config/constants.h` retains integration's 10.5-inch track
  width and adds the incoming distance-sensor ports. `dev/odomUpdates` used a
  2-inch track-width placeholder; the physical value still needs verification.
- Integration's Brain startup, Brain code, style guide and serial protocol are
  retained. The incoming Brain-side odometry task startup is not wired into
  the Nano. The unrelated, unconfigured `rplidar_sdk` gitlink is excluded.
  Clones that previously checked out `dev/odomUpdates` with submodules may
  report `unable to rmdir 'jetson-nano/external/rplidar_sdk': Directory not
  empty` and show the directory as untracked. Git does not delete a
  submodule's files on checkout. Remove the leftover directory with
  `rm -rf jetson-nano/external/rplidar_sdk`; its Git data remains under
  `.git/modules/`.

## Deferred work

The relocated code still contains PROS hardware reads, tasks, mutexes, motor
commands, timing and diagnostics, as well as includes from the Brain-oriented
layout. The Nano does not yet receive the IMU and distance-sensor data needed
by these algorithms. No adapters, protocol changes, startup connections or
placeholder implementations were added in this merge.

The archived incoming `Autonomous` is not connected to the Nano entry point,
and the pose source in Nano `main.cpp` still returns no pose. The Nano CMake source lists have not been updated
to compile the relocated localization or controller files. The incoming code
uses C++ facilities and dependencies that need separate platform integration.
Consequently neither a successful existing target build nor this merge should
be taken as verification that the relocated code builds or operates.

The odometry PR also calls for physical checks of IMU direction, sensor failure
and recovery, and retuning turn/heading PID values for radians. These remain
future work.
