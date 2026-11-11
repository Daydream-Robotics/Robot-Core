#pragma once
#include <cstddef>
#include <vector>
#include "motion/control/motionController.hpp"
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
         * @param paths paths to run in order, every autonomous period
         * @param flags direction of each path, same order as paths
         */
        Autonomous(PathFollower& pathFollower, const SerialLink& link, const std::vector<ALS_Path>& paths, const std::vector<PathFlag>& flags);

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
         * @brief whether the current autonomous period is over
         * @returns true on shutdown, or when the brain reports it's no longer in an enabled autonomous.
         *          false while no state arrives (a link dropout pauses the routine)
         */
        bool autonOver();

        /**
         * @brief holds the drive stopped until an enabled autonomous starts (or shutdown)
         */
        void waitForAuton();

        /**
         * @brief holds the drive stopped until the autonomous period is over (or shutdown)
         */
        void waitForAutonEnd();

        /**
         * @brief follows one path to its end. a short link dropout pauses it, then it carries on
         * @returns false if autonomous ended or shutdown was requested first
         */
        bool runPath(std::size_t index);

        PathFollower& m_pathFollower; /**< drives the paths */
        const SerialLink& m_link; /**< competition state */
        const std::vector<ALS_Path>& m_paths; /**< paths, in run order */
        const std::vector<PathFlag>& m_flags; /**< direction of each path */
        Ticker m_ticker; /**< one tick per MPC_SAMPLE_PERIOD */
        bool m_linkLost = false; /**< no state from the brain right now, for the lost/back messages */
};
