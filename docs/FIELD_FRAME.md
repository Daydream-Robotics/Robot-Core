# Field Frame

The frame `main_ekf_` (`FullEkf`) runs in, and the frame the static field map (#144) is drawn in. Both must use this definition.

## Origin

(0, 0) is the center of the field.

## Axes

Same as jerryio. Units are inches.

- +X points forward, away from the red driver station. +Y points left of +X.
- A robot at heading 0 faces +Y, and its sideways (perpendicular wheel) direction is +X.
- Heading is compass: 0 points along +Y, 90° along +X, increasing clockwise.

## Alliance side

One frame for both alliances. The frame is tied to the red driver station, so it does not change with the alliance, and the static field map (#144) is drawn once.

Autons are written once, for red, and transformed for blue before they run: every path point, path heading, and the start pose.

This season (Override) the field is rotationally symmetric: rotating 180° about the field center takes the red driver station to the blue one, with every field element in the same place (red elements become blue). The blue transform is:

- (x, y) → (−x, −y)
- θ → θ + π, wrapped to (−π, π]

A rotation keeps clockwise as clockwise, so hand-written turns ("turn right 90°") stay the same for blue.

Do not mirror or rotate the frame itself. It stays tied to the red driver station for both alliances.

## Start poses

Each auton's start pose is the first point of its jerryio path, with that point's heading. Blue autons use the same pose rotated 180° (above), then `setPose()` with the start covariance below.

Start covariance is `FULL_EKF_INIT_XY_SIGMA` (1 in) and `FULL_EKF_INIT_THETA_SIGMA` (about 2°) in `constants.h`.

## odom_ekf_

`odom_ekf_` (`OdomEkf`) is not in the field frame and is not reset by `setPose()`. Its frame starts wherever it starts, and only its increments are used.
