#pragma once
#include <cstddef>
#include <vector>
#include "motion/pathFollower.hpp"
#include "motion/pathing/arclengthSplining.hpp"
#include "protocol/serial.hpp"
#include "utils/helpers.hpp"

/**
 * @class Autonomous
 * @brief runs the paths during each autonomous period the brain reports
 */
class Autonomous {
    public:
        /**
         * @param pathFollower follower to drive the paths with
         * @param link brain link, read for the competition state
         * @param paths paths to run in order, every autonomous period. must outlive this object
         */
        Autonomous(PathFollower& pathFollower, const SerialLink& link, const std::vector<ALS_Path>& paths);

        /**
         * @brief runs until shutdown is requested: waits for autonomous, runs every path, holds still until it ends
         * @note leaves the drive stopped when it returns
         */
        void run();

    private:
        /**
         * @brief whether the brain reports an enabled autonomous period
         */
        bool enabled() const;

        /**
         * @brief holds the drive stopped while enabled() == whileEnabled (and no shutdown)
         */
        void holdStopped(bool whileEnabled);

        /**
         * @brief follows one path to its end
         * @returns false if autonomous ended or shutdown was requested first
         */
        bool runPath(std::size_t index);

        PathFollower& m_pathFollower; /**< drives the paths */
        const SerialLink& m_link; /**< competition state */
        const std::vector<ALS_Path>& m_paths; /**< paths, in run order */
        Ticker m_ticker; /**< one tick per MPC_SAMPLE_PERIOD */
};
