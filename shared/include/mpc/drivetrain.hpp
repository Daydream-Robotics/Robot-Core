#ifndef _DRIVETRAIN_HPP_
#define _DRIVETRAIN_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

//drivetrain kinematics for the MPC, header-only and std only so the brain can include it too
//a motor group is a set of motors that always gets the same voltage and spins at one speed
//(e.g. one side of a diff or H drive), the MPC models each group as a single motor
//body twist is (vx, vy, w): vx forward, vy left, w counterclockwise (rad/s), the MPC converts it to the Pose convention
namespace kinematics {
    enum class DriveMode : uint8_t { DIFF = 0, X = 1, H = 2 };

    template<std::size_t R, std::size_t C>
    using Mat = std::array<std::array<double, C>, R>;

//geometry, each drive reads only the fields it needs
struct DriveGeometry {
    double wheelRadius = 0.0; //drive wheels (in)
    double trackWidth = 0.0; //diff and H (in)
    double rho = 0.0; //X, center to wheel distance (in)
    double strafeRadius = 0.0; //H, strafe wheel (in)
    double strafeOffset = 0.0; //H, strafe wheel forward offset from center (in)
};

//group order: L, R
struct DiffDrive {
    static constexpr DriveMode MODE = DriveMode::DIFF;
    static constexpr std::size_t N_GROUPS = 2;
    static constexpr bool HOLONOMIC = false;

    //true when every geometry field this drive divides by is positive
    static bool valid(const DriveGeometry& g) {
        return g.wheelRadius > 0.0 && g.trackWidth > 0.0;
    }
    //group speeds from body twist: omega = G * (vx, vy, w)
    static Mat<2, 3> G(const DriveGeometry& g) {
        const double r = g.wheelRadius, L = g.trackWidth;
        return {{{1.0 / r, 0.0, -L / (2.0 * r)},
                 {1.0 / r, 0.0,  L / (2.0 * r)}}};
    }
    //body twist from group speeds: (vx, vy, w) = K * omega
    static Mat<3, 2> K(const DriveGeometry& g) {
        const double r = g.wheelRadius, L = g.trackWidth;
        return {{{r / 2.0, r / 2.0},
                 {0.0, 0.0},
                 {-r / L, r / L}}};
    }
};

//group order: FL, FR, BL, BR, rollers at 45 deg, positive is forward on every wheel
//matches the mixing in docs/X_DRIVE.md (strafe and turn there are right and clockwise)
struct XDrive {
    static constexpr DriveMode MODE = DriveMode::X;
    static constexpr std::size_t N_GROUPS = 4;
    static constexpr bool HOLONOMIC = true;

    static bool valid(const DriveGeometry& g) {
        return g.wheelRadius > 0.0 && g.rho > 0.0;
    }

    static Mat<4, 3> G(const DriveGeometry& g) {
        const double c = 1.0 / std::sqrt(2.0), p = g.rho, r = g.wheelRadius;
        return {{{c / r, -c / r, -p / r},
                 {c / r,  c / r,  p / r},
                 {c / r,  c / r, -p / r},
                 {c / r, -c / r,  p / r}}};
    }
    //columns of r*G are orthogonal, so K = r * diag(1/2, 1/2, 1/(4 rho^2)) * (r*G)^T
    static Mat<3, 4> K(const DriveGeometry& g) {
        const auto Gm = G(g);
        const double r = g.wheelRadius, p = g.rho;
        Mat<3, 4> k{};
        for (std::size_t i = 0; i < 4; i++) {
            k[0][i] = r * (r * Gm[i][0]) / 2.0;
            k[1][i] = r * (r * Gm[i][1]) / 2.0;
            k[2][i] = r * (r * Gm[i][2]) / (4.0 * p * p);
        }
        return k;
    }
};

//group order: L, R, strafe
struct HDrive {
    static constexpr DriveMode MODE = DriveMode::H;
    static constexpr std::size_t N_GROUPS = 3;
    static constexpr bool HOLONOMIC = true;

    static bool valid(const DriveGeometry& g) {
        return g.wheelRadius > 0.0 && g.trackWidth > 0.0 && g.strafeRadius > 0.0;
    }

    static Mat<3, 3> G(const DriveGeometry& g) {
        const double rs = g.wheelRadius, rc = g.strafeRadius;
        const double L = g.trackWidth, xc = g.strafeOffset;
        return {{{1.0 / rs, 0.0, -L / (2.0 * rs)},
                 {1.0 / rs, 0.0,  L / (2.0 * rs)},
                 {0.0, 1.0 / rc, xc / rc}}};
    }
    //K = H^-1 * diag(rs, rs, rc)
    static Mat<3, 3> K(const DriveGeometry& g) {
        const double rs = g.wheelRadius, rc = g.strafeRadius;
        const double L = g.trackWidth, xc = g.strafeOffset;
        return {{{rs / 2.0, rs / 2.0, 0.0},
                 {rs * xc / L, -rs * xc / L, rc},
                 {-rs / L, rs / L, 0.0}}};
    }
};

//largest group count of any drive, sizes per-group arrays that have to fit every drive (e.g. serial packets)
constexpr std::size_t MAX_MOTOR_GROUPS = std::max({DiffDrive::N_GROUPS, XDrive::N_GROUPS, HDrive::N_GROUPS});

}
#endif