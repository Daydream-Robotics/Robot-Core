#pragma once

// Math helpers shared by the brain and the Nano. Header-only and std only (no PROS, no Eigen).

namespace utils {

/**
 * @brief Wraps an angle to (-pi, pi].
 *
 * @param a Angle in radians.
 * @return double The same angle in (-pi, pi].
 */
inline double wrapAngle(double a) {
    constexpr double PI = 3.14159265358979323846;
    while (a > PI) a -= 2.0 * PI;
    while (a < -PI) a += 2.0 * PI;
    return a;
}

} // namespace utils
