#include "LRQ.hpp"
#include  "helpers.hpp"
#include <algorithm>
#include <cmath>
#include <array>
#include <numbers>

//* ----------- HELPER FUNCTIONS ---------------------------------------------------------------------

namespace {
    //* Constants
    constexpr double kMaxMotorRPM = 600.0; // Maximum RPM of the motors
    constexpr double kMinDeterminant = 1e-9; // Minimum determinant to avoid singularity
    constexpr int kDareIterations = 100; // Number of iterations for DARE solver
    constexpr double kDareTolerance = 1e-9; // Tolerance for DARE solver convergence

    //* Type aliases for matrices and vectors
    using Matrix3 = std::array<std::array<double, 3>, 3>;
    using Matrix2 = std::array<std::array<double, 2>, 2>;
    using Matrix3x2 = std::array<std::array<double, 2>, 3>;
    using Matrix2x3 = std::array<std::array<double, 3>, 2>;
    using Vector3 = std::array<double, 3>;
    using Vector2 = std::array<double, 2>;



    //* ------------ MATRIX OPERATIONS ---------------------------------------------------------------------

    // Create a 3x3 zero matrix
    Matrix3 zero3() {
        return {{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}};
    }

    // Create a 2x2 zero matrix
    Matrix2 zero2() {
        return {{{0.0, 0.0}, {0.0, 0.0}}};
    }

    // Create a 3x3 identity matrix
    Matrix3 identity3() {
        Matrix3 result = zero3();
        result[0][0] = 1.0;
        result[1][1] = 1.0;
        result[2][2] = 1.0;
        return result;
    }

    // Transpose a 3x3 matrix
    Matrix3 transpose(const Matrix3& mat) {
        Matrix3 result = zero3();

        for (std::size_t i = 0; i < 3; i++) {
            for (std::size_t j = 0; j < 3; j++) {
                result[i][j] = mat[j][i];
            }
        }
        return result;
    }

    // Transpose a 3x2 matrix to a 2x3 matrix
    Matrix2x3 transpose(const Matrix3x2& mat) {
        Matrix2x3 result = {{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}};

        for (std::size_t i = 0; i < 3; i++) {
            for (std::size_t j = 0; j < 2; j++) {
                result[j][i] = mat[i][j];
            }
        }
        return result;
    }

    // Transpose a 2x3 matrix to a 3x2 matrix
    Matrix3x2 transpose(const Matrix2x3& matrix) {
        Matrix3x2 result = {{{0.0, 0.0},
                            {0.0, 0.0},
                            {0.0, 0.0}}};

        for (int row = 0; row < 2; row++) {
            for (int col = 0; col < 3; col++) {
                result[col][row] = matrix[row][col];
            }
        }

        return result;
    }

    // Multiply a 3x3 matrix by a 3x3 matrix
    Matrix3 multiply(const Matrix3& lhs, const Matrix3& rhs) {
        Matrix3 result = zero3();

        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                for (int k = 0; k < 3; k++) {
                    result[row][col] += lhs[row][k] * rhs[k][col];
                }
            }
        }

        return result;
    }

    // Multiply a 3x3 matrix by a 3x2 matrix
    Matrix3x2 multiply(const Matrix3& lhs, const Matrix3x2& rhs) {
        Matrix3x2 result = {{{0.0, 0.0},
                            {0.0, 0.0},
                            {0.0, 0.0}}};

        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 2; col++) {
                for (int k = 0; k < 3; k++) {
                    result[row][col] += lhs[row][k] * rhs[k][col];
                }
            }
        }

        return result;
    }

    // Multiply a 2x3 matrix by a 3x3 matrix
    Matrix2x3 multiply(const Matrix2x3& lhs, const Matrix3& rhs) {
        Matrix2x3 result = {{{0.0, 0.0, 0.0},
                            {0.0, 0.0, 0.0}}};

        for (int row = 0; row < 2; row++) {
            for (int col = 0; col < 3; col++) {
                for (int k = 0; k < 3; k++) {
                    result[row][col] += lhs[row][k] * rhs[k][col];
                }
            }
        }

        return result;
    }

    // Multiply a 2x2 matrix by a 2x3 matrix
    Matrix2x3 multiply(const Matrix2& lhs, const Matrix2x3& rhs) {
        Matrix2x3 result = {{{0.0, 0.0, 0.0},
                            {0.0, 0.0, 0.0}}};

        for (int row = 0; row < 2; row++) {
            for (int col = 0; col < 3; col++) {
                for (int k = 0; k < 2; k++) {
                    result[row][col] += lhs[row][k] * rhs[k][col];
                }
            }
        }

        return result;
    }

    // Multiply a 2x3 matrix by a 3x2 matrix
    Matrix2 multiply(const Matrix2x3& lhs, const Matrix3x2& rhs) {
        Matrix2 result = zero2();

        for (int row = 0; row < 2; row++) {
            for (int col = 0; col < 2; col++) {
                for (int k = 0; k < 3; k++) {
                    result[row][col] += lhs[row][k] * rhs[k][col];
                }
            }
        }

        return result;
    }

    // Multiply a 3x2 matrix by a 2x3 matrix
    Matrix3 multiply(const Matrix3x2& lhs, const Matrix2x3& rhs) {
        Matrix3 result = zero3();

        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                for (int k = 0; k < 2; k++) {
                    result[row][col] += lhs[row][k] * rhs[k][col];
                }
            }
        }

        return result;
    }

    // Multiply a 2x3 matrix by a 3x1 vector
    Vector2 multiply(const Matrix2x3& lhs, const Vector3& rhs) {
        Vector2 result = {0.0, 0.0};

        for (int row = 0; row < 2; row++) {
            for (int k = 0; k < 3; k++) {
                result[row] += lhs[row][k] * rhs[k];
            }
        }

        return result;
    }

    // Add two 3x3 matrices
    Matrix3 add(const Matrix3& lhs, const Matrix3& rhs) {
        Matrix3 result = zero3();

        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                result[row][col] = lhs[row][col] + rhs[row][col];
            }
        }

        return result;
    }

    // Add two 2x2 matrices
    Matrix2 add(const Matrix2& lhs, const Matrix2& rhs) {
        Matrix2 result = zero2();

        for (int row = 0; row < 2; row++) {
            for (int col = 0; col < 2; col++) {
                result[row][col] = lhs[row][col] + rhs[row][col];
            }
        }

        return result;
    }

    // Subtract two 3x3 matrices
    Matrix3 subtract(const Matrix3& lhs, const Matrix3& rhs) {
        Matrix3 result = zero3();

        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                result[row][col] = lhs[row][col] - rhs[row][col];
            }
        }

        return result;
    }

    // Multiply a 3x3 matrix by a scalar
    Matrix3 scale(const Matrix3& matrix, double scalar) {
        Matrix3 result = zero3();

        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                result[row][col] = matrix[row][col] * scalar;
            }
        }

        return result;
    }

    // Compute the maximum absolute difference between two 3x3 matrices
    double maxAbsDifference(const Matrix3& lhs, const Matrix3& rhs) {
        double result = 0.0;

        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                result = std::max(result, std::abs(lhs[row][col] - rhs[row][col]));
            }
        }

        return result;
    }

    // 2x2 matrix inversion using the formula for 2x2 matrices
    bool invert(const Matrix2& matrix, Matrix2& inverse) {
        const double determinant =
            matrix[0][0] * matrix[1][1] -
            matrix[0][1] * matrix[1][0];

        if (std::abs(determinant) < kMinDeterminant) {
            inverse = zero2();
            return false;
        }

        const double invDeterminant = 1.0 / determinant;

        inverse[0][0] =  matrix[1][1] * invDeterminant;
        inverse[0][1] = -matrix[0][1] * invDeterminant;
        inverse[1][0] = -matrix[1][0] * invDeterminant;
        inverse[1][1] =  matrix[0][0] * invDeterminant;

        return true;
    }



    //* ----------- LQR SOLVER ---------------------------------------------------------------------

    // Solve the Discrete-time Algebraic Riccati Equation (DARE) iteratively
    bool solveDare(
        const Matrix3& a,
        const Matrix3x2& b,
        const Matrix3& q,
        const Matrix2& r,
        Matrix3& p
    ) {
        p = q;

        const Matrix3 aTranspose = transpose(a);
        const Matrix2x3 bTranspose = transpose(b);

        for (int i = 0; i < kDareIterations; i++) {
            const Matrix3 previousP = p;

            const Matrix3x2 pB = multiply(p, b);
            const Matrix2 bTransposePB = multiply(bTranspose, pB);
            const Matrix2 s = add(r, bTransposePB);

            Matrix2 sInverse = zero2();
            if (!invert(s, sInverse)) {
                p = previousP;
                return false;
            }

            const Matrix3 pA = multiply(p, a);
            const Matrix2x3 bTransposePA = multiply(bTranspose, pA);
            const Matrix2x3 feedbackTerm = multiply(sInverse, bTransposePA);

            const Matrix3x2 aTransposePB = multiply(aTranspose, pB);
            const Matrix3 correction = multiply(aTransposePB, feedbackTerm);
            const Matrix3 nextP = add(subtract(multiply(aTranspose, pA), correction), q);

            p = nextP;

            if (maxAbsDifference(p, previousP) < kDareTolerance) {
                return true;
            }
        }

        return true;
    }

    // Compute the LQR gain matrix K, given system matrices A, B, Q, R, and solution P to the DARE
    bool computeGain(
        const Matrix3& a,
        const Matrix3x2& b,
        const Matrix2& r,
        const Matrix3& p,
        Matrix2x3& k
    ) {
        const Matrix2x3 bTranspose = transpose(b);
        
        const Matrix3x2 pB = multiply(p, b);
        const Matrix2 s = add(r, multiply(bTranspose, pB));

        Matrix2 sInverse = zero2();
        if (!invert(s, sInverse)) {
            k = {{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}};
            return false;
        }

        const Matrix3 pA = multiply(p, a);
        const Matrix2x3 bTransposePA = multiply(bTranspose, pA);

        k = multiply(sInverse, bTransposePA);
        return true;
    }

    //* ----------- LQR CONTROLLER SETUP ---------------------------------------------------------------------

    // Build the Q matrix from the LQRConfig
    Matrix3 buildQ(const LQRConfig& config) {
        Matrix3 q = zero3();

        q[0][0] = config.qX;
        q[1][1] = config.qY;
        q[2][2] = config.qTheta;

        return q;
    }

    // Build the R matrix from the LQRConfig
    Matrix2 buildR(const LQRConfig& config) {
        Matrix2 r = zero2();

        r[0][0] = config.rV;
        r[1][1] = config.rOmega;

        return r;
    }

    // Build the discrete-time A matrix based on target linear and angular velocities and time step
    Matrix3 buildDiscreteA(double targetLinearVel, double targetAngularVel, double dt) {
        Matrix3 a = identity3();

        a[0][1] = targetAngularVel * dt;
        a[0][2] = -targetAngularVel * dt;
        a[1][2] = targetLinearVel * dt;

        return a;
    }

    // Build the discrete-time B matrix based on time step
    Matrix3x2 buildDiscreteB(double dt) {
        return {{{dt, 0.0}, {0.0, 0.0}, {0.0, dt}}};
    }

    //* ----------- HELPER FUNCTIONS FOR WHEEL VELOCITIES ---------------------------------------------------------------------

    double inchesPerSecondToMotorRpm(double inchesPerSecond) {
        const double wheelCircumference = std::numbers::pi * DRIVE_WHEEL_DIAMETER_INCHES;

        if (wheelCircumference <= 0.0) {
            return 0.0;
        }

        return inchesPerSecond * 60.0 / wheelCircumference;
    }

    WheelVelocities chassisSpeedsToWheelVelocities(
        double linearVelocity,
        double angularVelocity,
        double trackWidthInches
    ) {
        const double halfTrackWidth = trackWidthInches * 0.5;

        const double leftIps = linearVelocity - angularVelocity * halfTrackWidth;
        const double rightIps = linearVelocity + angularVelocity * halfTrackWidth;

        const double leftRpm = std::clamp(
            inchesPerSecondToMotorRpm(leftIps),
            -kMaxMotorRPM,
            kMaxMotorRPM
        );

        const double rightRpm = std::clamp(
            inchesPerSecondToMotorRpm(rightIps),
            -kMaxMotorRPM,
            kMaxMotorRPM
        );

        return WheelVelocities{leftRpm, rightRpm, ControlMode::INPUT_VELOCITY};
    }
}

LQRController::LQRController(LQRConfig config)
    : m_config(config) {}

void LQRController::reset() {}
