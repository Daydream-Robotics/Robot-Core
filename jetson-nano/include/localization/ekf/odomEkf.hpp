#pragma once

#include <Eigen/Dense>

#include "localization/odometry.hpp"

// one cycle of odometry increments, defined by the odometry input adapter
struct OdomInputs;

/**
 * @brief Odometry EKF: fuses tracking wheels, IMEs, and IMU. Takes relative movements.
 *
 * State is (x, y, theta, v_x, v_y, omega). Pose is in the odometry frame, and v_x, v_y are
 * forward and sideways speed in the robot frame.
 */
class OdomEkf {
public:
    using State = Eigen::Matrix<double, 6, 1>; ///< x, y, theta, v_x, v_y, omega
    using Cov = Eigen::Matrix<double, 6, 6>;

    /**
     * @brief Constructs the filter at the origin with zero velocity.
     */
    OdomEkf() = default;

    /**
     * @brief Runs one predict + update cycle.
     *
     * @param in This cycle's increments and which sources are valid.
     */
    void step(const OdomInputs& in);

    /**
     * @brief Latest state.
     *
     * @return const State& (x, y, theta, v_x, v_y, omega).
     */
    const State& state() const;

    /**
     * @brief Latest state covariance.
     *
     * @return const Cov& 6x6 covariance, same order as the state.
     */
    const Cov& covariance() const;

    /**
     * @brief Sets the pose and zeroes the velocity (link reconnect). The odometry-frame origin is arbitrary.
     *
     * @param pose New pose in the odometry frame.
     */
    void reset(const Pose& pose);

private:
    static constexpr int MAX_ROWS = 5; ///< parallel, perpendicular, two IMEs, IMU

    using MeasVec = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, MAX_ROWS, 1>; ///< up to MAX_ROWS x 1
    using MeasMat = Eigen::Matrix<double, Eigen::Dynamic, 6, 0, MAX_ROWS, 6>; ///< up to MAX_ROWS x 6
    using MeasCov = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, MAX_ROWS, MAX_ROWS>; ///< up to MAX_ROWS x MAX_ROWS
    using Gain = Eigen::Matrix<double, 6, Eigen::Dynamic, 0, 6, MAX_ROWS>; ///< 6 x up to MAX_ROWS

    /**
     * @brief Stacked measurement for one cycle: only the rows for valid sources.
     */
    struct Measurement {
        MeasVec z; ///< measured increments, one row per valid source
        MeasMat H; ///< maps the state to each row
        MeasCov R; ///< measurement noise, diagonal
    };

    /**
     * @brief Moves the state forward by dt with constant velocities and grows P by the process noise.
     *
     * @param dt Seconds since the last step.
     */
    void predict(double dt);

    /**
     * @brief Builds z, H, and R from the valid sources in this cycle's inputs.
     *
     * @param in This cycle's increments.
     * @return Measurement The stacked rows. Empty if no source is valid.
     */
    Measurement buildMeasurement(const OdomInputs& in) const;

    /**
     * @brief Corrects the state with one stacked measurement (Joseph-form covariance update).
     *
     * @param m Measurement from buildMeasurement().
     */
    void update(const Measurement& m);

    /**
     * @brief Covariance at startup, after reset(), and for the velocities after a link stall.
     *
     * Pose variances are tiny because the odometry frame starts where the filter starts.
     * Velocity std devs are the robot's top speeds (60 in/s, 4 rad/s), because the robot may be
     * moving at full speed when reset() runs on a link reconnect.
     */
    static inline const Cov initialP = (State() << 1e-4, 1e-4, 1e-6, 3600.0, 3600.0, 16.0).finished().asDiagonal().toDenseMatrix();

    State m_s = State::Zero(); ///< starts at the odometry-frame origin, at rest
    Cov m_P = initialP; ///< same start as reset()
};
