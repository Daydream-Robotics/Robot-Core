# Field Frame

The frame `main_ekf_` (`FullEkf`) runs in, and the frame the static field map (#144) is drawn in. Both must use this definition.

## Origin

(0, 0) is the center of the field.

## Axes

Same as jerryio. Units are inches.

- +X points forward (TODO: toward which field wall), +Y points left of +X.
- Robot frame: +Y forward, +X right, so a robot at heading 0 lines up with the field axes.
- Heading is compass: 0 points along +Y, 90° along +X, increasing clockwise.


## Alliance side

TODO: not decided yet. Either one frame for both alliances (start poses differ), or a frame mirrored per alliance (start poses match, the map is mirrored).

## Start poses

TODO: the `setPose()` pose for each auton start position, for both alliances.

| Alliance | Start position | x (in) | y (in) | heading (compass, deg) |
|---|---|---|---|---|
| Red | TODO | | | |
| Blue | TODO | | | |

Start covariance is `FULL_EKF_INIT_XY_SIGMA` (1 in) and `FULL_EKF_INIT_THETA_SIGMA` (about 2°) in `constants.h`.

## odom_ekf_

`odom_ekf_` (`OdomEkf`) is not in the field frame and is not reset by `setPose()`. Its frame starts wherever it starts, and only its increments are used.
