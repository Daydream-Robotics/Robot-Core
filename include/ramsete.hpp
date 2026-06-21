#pragma once
#include "constants.h"
#include "odometry.hpp"
#include "motionController.hpp"

/**
 * @struct RamseteConfig
 * @brief Tuning constants and chassis measurements for the ramesete controller
 */
struct RamseteConfig {
    /**
     * @brief Proportional tracking spring constant
     * @note Higher values causes the robot to converge to the path more agressively but lower damping stability and can cause oscilations.
     */
    double b;

    /**
     * @brief Daming ratio
     * @note Range: (0 < zeta < 1)
     * @note Higher values will supress path convergence smoothing out oscilations.
     */
    double zeta;

    /** track distance between left and right weheels in inches */
    double trackWidthInches; 
};

/**
 * @class Ramsete Controller
 * @brief real-time non-linear feedback controller
 */
class RamseteController : public MotionController {
    private:
        double m_b;                 /**< Proportional paremeter b */
        double m_zeta;              /**< Damping parameter zeta */
        double m_trackWidthInches;  /**< drivetrain width in inches */

        /**
         * @brief computes sin(x)/x with divide by zero catch
         * @param x angle in radians
         * @note Includes catch to prevent divide by zero
         * @returns vlaue of sin(x)/x or 1.0 if x is near zero
         */
        double sinc(double x);

    public:
        /**
         * @brief Constructs Ramsete controller instance
         * @param config The config for the ramsete controller
         */
        RamseteController(RamseteConfig config);

        /**
         * @brief Constructs Ramsete controller instance
         * @param b Proportional paremeter
         * @param zeta Damping parameter
         * @param trackWidthInches drivetrain width in inches
         */
        RamseteController(double b, double zeta, double trackWidthInches);
        
        virtual ~RamseteController() = default;

        /**
         * @brief Computes the commanded wheel velocities
         * @param currentPose latest pose of the robot
         * @param als_path splined path with target samples
         * @param targetIdx lookup index of the closest sample to the robot
         * @param flag sets direction of tracking
         * @returns left and right wheel velocities 
         */
        WheelVelocities compute(const Pose& currentPose, const ALS_Path& als_path, std::size_t& targetIdx, PathFlag flag) override;
};