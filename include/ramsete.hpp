#pragma once
#include "constants.h"
#include "odometry.hpp"
#include "motionController.hpp"

struct RamseteConfig {
    double b;                   // proportional gain (higher means robot converges to path more agressively) default: 2
    double zeta;                // damping ratio (higher zeta smooths out oscillations but slows convergence) defaul: 0.7
    double trackWidthInches;
};

/**
 * @class Ramsete Controller
 */
class RamseteController : public MotionController {
    private:
        double m_b;
        double m_zeta;
        double m_trackWidthInches;

        /**
         * @brief computes sin(x)/x with divide by zero catch
         */
        double sinc(double x);

    public:
        /**
         * @brief Constructs Ramsete controller instance
         * @param config The config for the ramsete controller
         */
        RamseteController(RamseteConfig config);
        RamseteController(double b, double zeta, double trackWidthInches);
        
        virtual ~RamseteController() = default;

        /**
         * @brief Computes the commanded wheel velocities
         */
        WheelVelocities compute(const Pose& currentPose, const ALS_Path& als_path, std::size_t& closestSampleIdx, PathFlag flag) override;
};