#pragma once
#include <functional>
#include <optional>
#include "motion/pathing/arclengthSplining.hpp"
#include "motion/control/motionController.hpp"
#include "protocol/serial.hpp"

/**
 * @class PathFollower
 * @brief Manages following a trajectory
 * @note drive commands go to the brain through the SerialLink, measurements come from its latest Sample
 */
class PathFollower {
    public:
        /// @brief returns the robot's latest pose, or nothing if there isn't a good one
        using PoseSource = std::function<std::optional<Pose>()>;

        /**
         * @brief Initializes the tracker with a motion controller
         * @param controller Motion controller to use to compute motor speeds
         * @param link Brain link: drive speeds and battery in, drive commands out
         * @param poseSource Where the robot's pose comes from (see Pose for the frame)
         */
        PathFollower(MotionController& controller, SerialLink& link, PoseSource poseSource);
        ~PathFollower() = default;

        /**
         * @brief Loads new trajectory and resets parameters
         * @param path Splined path containing information along the path. must outlive the follower's use of it
         * @param flag Direction of travel
         */
        void setPath(const ALS_Path& path, PathFlag flag = PathFlag::FORWARDS);

        /**
         * @brief Executes a single clock step of the tracking controller
         * @returns Boolean true if path has reached the end of the path within tolerance
         */
        bool step();

        /**
         * @brief Stops both drive groups (STOP, using the group's brake mode)
         */
        void stop();

    private:
        /**
         * @brief Stops the drive and resets the controller, for when the drive was stopped mid-path
         */
        void stopAndReset();

        MotionController& m_controller;         /**< Reference to motion controller */
        SerialLink& m_link;                     /**< Brain link */
        PoseSource m_poseSource;                /**< Robot pose */
        const ALS_Path* m_path = nullptr;       /**< Pointer to current path being tracked */

        std::size_t m_currentSampleIdx = 0;     /**< index of the current closest sample on the path */
        double m_distanceFromEnd = 999.0;       /**< arc length to the end of the path in inches */
        bool m_isFinished = false;              /**< Boolean true if end of path reached */

        PathFlag flag = PathFlag::FORWARDS;     /**< Direction config parameter */
};
