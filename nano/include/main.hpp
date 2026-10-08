#pragma once

// The robos estimated location on the field.
// +x is forward, +y is left, and positive rotation is counterclockwise.
struct Pose {
    double x = 0.0;      // inches
    double y = 0.0;      // inches
    double theta = 0.0;  // radians
};
