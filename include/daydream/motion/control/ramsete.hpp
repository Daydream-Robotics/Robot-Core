#pragma once

#include "daydream/motion/control/motionController.hpp"
#include "daydream/motion/odometry.hpp"

struct RamseteConfig {
    double b;
    double zeta;
    double trackWidthInches;
};

/**
 * @class RamseteController
 * @brief Computes differential-drive commands using a Ramsete controller.
 */
class RamseteController {
    public:
        /**
         * @brief Constructs a Ramsete controller instance.
         * @param config The configuration for the Ramsete controller.
         */
        RamseteController(RamseteConfig config);

        /**
         * @brief Computes the commanded wheel velocities.
         */
        WheelVelocities compute(Pose currentPose, Pose targetPose, double targetLinearVel, double targetAngularVel);

    private:
        double m_b;
        double m_zeta;
        double m_trackWidthInches;

        /**
         * @brief Computes sin(x)/x with a divide-by-zero guard.
         */
        double sinc(double x);
};
