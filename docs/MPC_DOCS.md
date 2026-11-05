## Where it runs
The MPC runs entirely on the Nano. The brain is only an I/O server on the serial link (see SERIAL_PROTOCOL.md): every 10 ms it sends `STATE` (drive group speeds, battery voltage, status bits) and applies the Nano's `COMMAND`.

Each 20 ms (`MPC_SAMPLE_PERIOD`) during an enabled autonomous period, `Autonomous::run()` (jetson-nano/src/autonomous/autonomous.cpp) steps the `PathFollower`, which:
1. reads the latest `SerialLink::Sample` and the robot pose from `latestPose()`
2. converts the drive group speeds from motor output shaft to wheel (`× DRIVE_GEAR_RATIO`)
3. predicts the wheel speeds and pose to when the command will apply (`MotionController::predict()`): the speeds are as old as the sample, and the command goes out with the brain's next state, `MPC_COMMAND_DELAY` later. The MPC does this with its motor model and the voltage it last sent
4. finds the closest point on the path, searching at most `PATH_SEARCH_AHEAD` past the last progress
5. calls the motion controller (`MPC`: reference rollout + solve)
6. sends the left/right voltages as `GroupMode::VOLTAGE` commands

Outside autonomous, or without a pose, it sends `STOP`. A failed solve sends 0 V for that step.

Each path runs forwards or in reverse from the JerryIO file: when a path's start heading (where the robot faces) points more than 90° away from its direction of travel, the robot backs along it (`PathFlag::REVERSE`).

If the brain's `STATE` stops (link dropout), the Nano holds the drive and keeps its progress, then carries on with the same path when the link is back. Only a `STATE` saying autonomous is over (disabled, or not autonomous) ends the routine; the next autonomous starts again from the first path.

Path following is split like on the brain: `PathFollower` (jetson-nano/src/motion/pathFollower.cpp) tracks progress along the path and sends the commands, and any `MotionController` computes them. `MPCController` is the only one on the Nano. To try another controller, implement `MotionController::compute()` (and optionally `predict()`) and pass it to the `PathFollower` in main.cpp.

The MPC's feedforward voltage for each reference stage is the voltage that moves the wheels from that stage's reference speed to the next one's in one step (from the exact discrete motor model).

The battery limit is the battery voltage the brain measures under load (`STATE.battery_mV`), which already includes sag, so there is no internal-resistance or current term.

## Frame
- +x forward, +y left
- heading 0 along +y, 90° (π/2) along +x, so it increases turning from +y toward +x
- angles are in radians in code

This is JerryIO's frame, so the Nano loads JerryIO paths as they are (field centre origin, cm converted to in) and the file's start heading is used directly. The pose source must use the same frame. The MPC model is `x' = v sinθ`, `y' = v cosθ`, `θ' = r(ω_L − ω_R)/L`.

## Steps
All MPC constants are in shared/include/config/constants.h: the `[MPC]` section (horizons, sample period, motor model, reference limits, cost weights, voltage/field/speed limits) and `[ROBOT DIMS]` (DRIVE_WHEEL_DIAMETER_INCHES, TRACK_WIDTH, DRIVE_GEAR_RATIO). `MPCController::Params()` builds the controller from them.

1. Get the motor model a and b (the old brain-side `MPCSerial::identifyMotorModel()` is in archive/v5-brain/src/motion/control/mpcSerial.cpp) and set MPC_MOTOR_A and MPC_MOTOR_B
2. Check DRIVE_WHEEL_DIAMETER_INCHES, TRACK_WIDTH and DRIVE_GEAR_RATIO
3. If wanted change MPC_P_X, MPC_P_Y, MPC_P_THETA, MPC_Q_U, MPC_Q_ZERO_MULT, MPC_Q_FINAL_MULT, MPC_P_FINAL_MULT, MPC_DELTA_U_MAX, MPC_DELTA_U_MIN (tuning steps below)
4. Connect the pose source: make latestPose() in jetson-nano/src/main.cpp return the EKF's latest pose (frame above)
5. read over jetson-nano/README.md to set up the Nano, copy over jetson-nano/ and shared/ (or clone the repo there) and build
6. set the link and path file in the `[NANO]` section of shared/include/config/constants.h, then run `./build/MPCController` on the Nano
7. the brain starts its SerialLink in `initialize()` and hands the drive to the Nano in `autonomous()`; the Nano drives during autonomous

The brain and the Nano must be built from the same shared/ folder (packet layouts and the built-in link config). Changing only the `[MPC]` constants needs just the Nano rebuilt.


## Tuning:
### MPC_DELTA_U_MAX and MPC_DELTA_U_MIN (Slew Rate Limits) (V/step):
- If too high: The motors will receive instant $12\text{V}$ spikes on acceleration, causing the wheels to spin out, slip on the tiles, and draw excessive battery current.
- If too low: The robot will feel sluggish, taking too long to speed up or slow down.
- Tuning Policy: Start at $\pm 4.0\text{V}$. If you notice wheel slippage on startup or during hard turns, decrease this value to 3 or 2.5.

### MPC_Q_U (Control Input Voltage Penalty)
This is the penalty weight of change in control input.
- If too low: The robot handles path variations aggressively, but the wheels may oscillate violently because it costs the optimizer virtually nothing to swing from $+12\text{ V}$ to $-12\text{ V}$.
- If too high: The robot drives very smoothly but turns wide on corners and fails to correct small errors because it values maintaining constant voltage over tracking precision.
- Tuning Policy: Start at 0.01 and vary based on conditions above

### MPC_P_Y
- Start with pY = 15.0: If during straight runs the robot drifts laterally or ignores path displacement, increase pY by 5.

### MPC_P_THETA
- Start with p_theta = 100.0 If the robot tracks straight lines perfectly but cuts corners on sharp turns, increase by 50
