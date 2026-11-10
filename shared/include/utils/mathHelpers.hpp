#pragma once

#include <math.h>
// Math helpers shared by the brain and the Nano. Header-only and std only (no PROS, no Eigen).

namespace utils {

/**
 * @brief Wraps an angle to (-pi, pi].
 *
 * @param a Angle in radians.
 * @return double The same angle in (-pi, pi].
 */

constexpr double PI = 3.14159265358979323846;

inline double wrapAngle(double a) {
    a = std::remainder(a, 2.0 * PI);
    return (a <= -PI) ? a + 2.0 * PI : a;
}

} // namespace utils
