#pragma once

#include "localization/odometry.hpp"
#include "config/constants.h"

/**
 * @brief Adaptive ICP correspondence threshold.
 *
 * Tracks how far each ICP result moved from its initial guess and turns that
 * into the max match distance (tau) and robust-kernel scale (sigma) for ICP.
 */
class AdaptiveThreshold {
public:
    /**
     * @brief Constructs the threshold, starting at tau0. Defaults come from config/constants.h.
     *
     * @param tau0In Starting max match distance before any data (in).
     * @param deltaMinIn Corrections smaller than this are ignored (in).
     * @param rMaxIn Max range, used to turn a heading error into a distance (in).
     */
    explicit AdaptiveThreshold(double tau0In = LIDAR_TAU_0,
                               double deltaMinIn = LIDAR_DELTA_MIN,
                               double rMaxIn = LIDAR_R_MAX);

    /**
     * @brief Records how far ICP moved from its initial guess. Call after each successful ICP.
     *
     * @param tInit ICP initial guess (field frame).
     * @param tResult ICP result (field frame).
     */
    void update(const Pose& tInit, const Pose& tResult);

    /**
     * @brief Typical guess error. ICP's Geman–McClure scale is sigma() / 3.
     *
     * @return double sigma_t (in).
     */
    double sigma() const;
    /**
     * @brief Max distance for a scan point and map point to count as a match.
     *
     * @return double tau_t = 3 * sigma_t (in).
     */
    double tau() const;

    /**
     * @brief Clears the statistics back to the starting tau0 (call on pose reset, A10).
     */
    void reset();

private:
    double m_tau0{0.0};
    double m_deltaMin{0.0};
    double m_rMax{0.0};

    double m_sumSq{0.0};  // sum of δ² plus the seed
    int m_count{0};       // number of samples plus the seed
};
