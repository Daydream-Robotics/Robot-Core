#pragma once
#include "arclengthSplining.hpp"
#include "motionController.hpp"

/**
 * @class PathFollower
 * @brief Manages following a trajectory
 */
class PathFollower {
    public:
        /**
         * @brief Initializes the tracker with a motion controller
         * @param controller Motion controller to use to compute motor speeds 
         */
        PathFollower(MotionController& controller);
        ~PathFollower() = default;
        
        /**
         * @brief Loads new trajectory and resets parameters
         * @param path Splined path containing information along the path
         * @param flag Direction of travel
         */
        void setPath(ALS_Path& path, PathFlag flag = PathFlag::FORWARDS);
        
        /**
         * @brief Executes a single clock step of the tracking controller
         * @returns Boolean true if path has reached the end of the path within tolerance
         */
        bool step();


    private:
        MotionController& m_controller;         /**< Reference to motion controller */
        ALS_Path* m_path = nullptr;             /**< Pointer to current path being tracked */

        std::size_t m_currentSampleIdx = 0;     /**< index of the current closest sample on the path */
        double m_distanceFromEnd = 999.0;       /**< arc length to the end of the path in inches */
        bool m_isFinished = false;              /**< Boolean true if end of path reached */

        PathFlag flag = PathFlag::FORWARDS;     /**< Direction config parameter */

        std::size_t MAX_VIEWABLE_INDEX_AHEAD = 50;
};