#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "localization/ekf/se2.hpp"
#include "localization/odometry.hpp"

// one cycle of odometry increments, defined by the odometry input adapter
struct OdomInputs;

/**
 * @brief Velocity output for the full EKF and the MPC.
 */
struct Twist {
    int64_t tUs = 0; ///< Nano steady_clock time (us)
    double v = 0.0; ///< forward speed (in/s)
    double omega = 0.0; ///< turn rate (rad/s)
    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero(); ///< covariance of (v, omega), in²/s² and rad²/s²
};

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

    using PoseCallback = std::function<void(const StampedPose&)>;
    using IncrementCallback = std::function<void(const PoseIncrement&)>;

    /**
     * @brief Subscribes to the odometry-frame pose, called after every step (100 Hz).
     *
     * Feeds the LiDAR pose buffer. Every subscriber gets every sample, not only the latest.
     *
     * @param cb Called with the pose, its time, and the pose block of P.
     */
    void onPose(PoseCallback cb);

    /**
     * @brief Subscribes to the per-cycle increment, called after every step.
     *
     * The full EKF's prediction input.
     *
     * @param cb Called with this cycle's motion in the previous pose's frame and its covariance.
     */
    void onIncrement(IncrementCallback cb);

    /**
     * @brief Motion between two times, composed from the per-cycle increments (for KISS-ICP).
     *
     * Treats cycles as independent when combining covariance, which is an approximation.
     *
     * @param t0Us Start time, Nano steady_clock (us).
     * @param t1Us End time, Nano steady_clock (us).
     * @return std::optional<PoseIncrement> Nothing if the span isn't in the buffer or crosses a reset().
     */
    std::optional<PoseIncrement> increment(int64_t t0Us, int64_t t1Us) const;

    /**
     * @brief Latest forward speed and turn rate.
     *
     * @return Twist v_x and omega from the state, with their 2x2 block of P.
     */
    Twist twist() const;

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
     * @brief This cycle's increment, from the pose before predict() to the pose after update().
     *
     * @param before State at the start of the step.
     * @param t0Us Start of the step, Nano steady_clock (us).
     * @return PoseIncrement Motion in before's frame, with this step's covariance only.
     */
    PoseIncrement stepIncrement(const State& before, int64_t t0Us) const;

    /**
     * @brief Sends the pose and the increment to every subscriber and stores the increment.
     *
     * @param inc This cycle's increment from stepIncrement().
     */
    void publish(const PoseIncrement& inc);

    static constexpr int INCREMENT_HISTORY = 100; ///< increments kept for increment(t0, t1), 1 s at 100 Hz

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
    Cov m_Pstep = Cov::Zero(); ///< covariance of this cycle's motion only, pose part starts at zero each step

    int64_t m_tUs = 0; ///< time of the latest step, Nano steady_clock (us)

    std::vector<PoseCallback> m_poseCallbacks; ///< onPose() subscribers
    std::vector<IncrementCallback> m_incrementCallbacks; ///< onIncrement() subscribers

    // TODO: per-step increment = between(previous pose, new pose), with its own covariance
    // TODO: keep a short ring buffer of increments so increment(t0, t1) can be composed on request
    std::array<PoseIncrement, INCREMENT_HISTORY> m_increments{}; ///< ring buffer, oldest overwritten first
    int m_incrementHead = 0; ///< index the next increment is written to
    int m_incrementCount = 0; ///< how many of m_increments are filled
};
