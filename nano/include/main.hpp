#pragma once

// Robot pose in the global field coordinate frame.
// +X = forward at heading 0
// +Y = left
// Positive rotation = counterclockwise

struct Pose {
    double x = 0.0;       // Inches
    double y = 0.0;       // Inches
    double theta = 0.0;   // Radians
};



