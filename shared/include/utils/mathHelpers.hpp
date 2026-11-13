#pragma once

#include <math.h>
// Math helpers shared by the brain and the Nano. Header-only and std only (no PROS, no Eigen).
// M_PI is not guaranteed by standard C++.
// Provide a fallback for compilers that don't define it.
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace utils {

/**
 * @brief Wraps an angle to (-pi, pi].
 *
 * @param a Angle in radians.
 * @return double The same angle in (-pi, pi].
 */
inline double wrapAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

} // namespace utils
