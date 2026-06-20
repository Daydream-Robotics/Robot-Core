#pragma once
#include "arclengthSplining.hpp"
#include "motionController.hpp"

class PathFollower {
    public:
        PathFollower(MotionController& controller);
        ~PathFollower() = default;

        enum Flags {
            FORWARDS,
            REVERSE
        };
        
        void setPath(ALS_Path& path, Flags flag = FORWARDS);
        bool step();


    private:
        MotionController& m_controller;
        ALS_Path* m_path = nullptr;

        std::size_t m_currentSampleIdx = 0;
        double m_distanceFromEnd = 999.0;
        bool m_isFinished = false;

        Flags flag = FORWARDS;
};