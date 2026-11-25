# LiDAR / EKF Assumptions

Everything the LiDAR code assumes about the EKFs. The LiDAR code only talks to the EKFs through `EkfInterface` (`jetson-nano/include/localization/ekf/ekfInterface.hpp`), and each assumption below is tagged on the interface member that relies on it.

- **From dev/EKF code** is what the EKF code on `dev/EKF` says.
- **Actual** is checked on the robot in #154.
- If an Actual answer differs, change the adapter, not the LiDAR code.

| ID | Assumption we code against | From dev/EKF code | Actual (#154) |
|---|---|---|---|
| A1 | EKF stamps use the Nano's `CLOCK_MONOTONIC`, the same clock as the LiDAR drivers | Yes. The EKFs stamp with `std::chrono::steady_clock` (us), which is `CLOCK_MONOTONIC` on Linux. The LiDAR driver reads `CLOCK_MONOTONIC` directly (`lidarDriver.cpp`) | ⬜ |
| A2 | `odom_ekf_` publishes at ≥ 100 Hz | 100 Hz (`OdomEkf`, `INCREMENT_HISTORY` is 1 s at 100 Hz) | ⬜ rate: ____ Hz |
| A3 | We receive every `odom_ekf_` sample (callback or queue), not only the latest value | Yes, `OdomEkf::onPose()` calls every subscriber with every sample | ⬜ mechanism: ____ |
| A4 | `odom_ekf_` never includes LiDAR measurements | Yes, `OdomEkf` only takes `OdomInputs` (encoders and IMU) | ⬜ |
| A5 | Poses are in inches/rad, +X forward, +Y left, heading 0 along +Y and clockwise positive, and the reference point is the robot's geometric center | Yes, the EKFs use this compass frame (`docs/FIELD_FRAME.md`, #189). Reference point is not stated | ⬜ reference point: ____ |
| A6 | `main_ekf_` is in a field-fixed frame that matches the static field map (#144): origin, axes, and alliance side | Yes (`docs/FIELD_FRAME.md`, #189): origin at the field center, +X away from the red driver station, one frame for both alliances | ⬜ origin: ____ axes: ____ |
| A7 | `main_ekf_` can return its pose at a past time `t`. If not, we fall back to `mainPoseLatest()` and reconstruct it (#151) | Yes, `FullEkf::poseAt()` over the last ~1 s | ⬜ yes / no |
| A8 | The measurement is accepted as an absolute pose with a 3×3 covariance ordered (x, y, θ), in in² and rad² | Yes, `PoseMeasurement` in `fullEkf.hpp` | ⬜ format: ____ |
| A9 | `main_ekf_` handles a measurement stamped `t_ref` that arrives late. We measure the latency and report it | Yes, `FullEkf::applyLate()` applies it at its stamp and replays the cycles since. Up to `FULL_EKF_HISTORY_SIZE` cycles (~1 s) late, older ones are dropped and counted in `tooOldCount()` | ⬜ method: ____ |
| A10 | We are notified when the pose is set (auton start, `setPose()`), and given the new pose | Yes, `FullEkf::onPoseReset()` is called once per accepted `FullEkf::setPose()` with the stored pose (#189). `OdomEkf::reset()` does not notify anyone | ⬜ mechanism: ____ |
| A11 | Alliance color and starting side are known to the EKF frame, so the field map does not need to be mirrored on our side | Yes. The frame is the same for both alliances, so the map is never mirrored. Blue autons are the red ones rotated 180° about the field center ((x, y) → (−x, −y), θ → θ + π), done before `setPose()` | ⬜ |
| A12 | `odom_ekf_` covariance is available. It is optional, and used only for diagnostics | Yes, `OdomEkf::onPose()` publishes the pose block of its covariance | ⬜ yes / no |

## Open items

- **A5:** confirm the pose reference point is the robot's geometric center.
- **A6, A11:** confirm the #144 map is drawn in this frame, with +X away from the red driver station.
- **A10:** the adapter forwards `FullEkf::onPoseReset()` to `EkfInterface::onPoseReset()`.
- **Time types:** the EKFs and `EkfInterface` use `int64_t` us, `LidarDriver` (`stampUs`) and `PoseBuffer` use `uint64_t` us. Cast at the boundary for now, make them match later.
