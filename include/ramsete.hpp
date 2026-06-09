#pragma once
#include "constants.h"
#include "odometry.hpp"

struct RamseteConfig {
    double b;
    double zeta;
    double trackWidthInches;
};

struct WheelVelocities {
    double left;
    double right;
};

/**
 * @class Ramsete Controller
 */
class RamseteController {
    public:
        /**
         * @brief Constructs Ramsete controller instance
         * @param config The config for the ramsete controller
         */
        RamseteController(RamseteConfig config);

        /**
         * @brief Computes the commanded wheel velocities
         */
        WheelVelocities compute(Pose currentPose, Pose targetPose, double targetLinearVel, double targetAngularVel);

    private:
        double m_b;
        double m_zeta;
        double m_trackWidthInches;

        /**
         * @brief computes sin(x)/x with divide by zero catch
         */
        double sinc(double x);
};