#pragma once
#include "arclengthSplining.hpp"
#include "motionController.hpp"
#include "fieldLogger.hpp"
#include "fieldLogger.hpp"

class PathFollower {
    public:
        PathFollower(MotionController& controller);
        ~PathFollower() = default;
        void setPath(ALS_Path& path, PathFlag flag = PathFlag::FORWARDS, bool logging = false, const char* baseName = "noname");
        bool step();

    private:
        MotionController& m_controller;         /**< Reference to motion controller */
        ALS_Path* m_path = nullptr;             /**< Pointer to current path being tracked */

        std::size_t m_currentSampleIdx = 0;
        double m_distanceFromEnd = 999.0;
        bool m_isFinished = false;

        PathFlag flag = PathFlag::FORWARDS;     /**< Direction config parameter */
        std::optional<FieldLogger> path_log;
        bool logging;
};