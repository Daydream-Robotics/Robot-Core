#ifndef _MPC_REFERENCE_HPP_
#define _MPC_REFERENCE_HPP_

#include "motion/pathing/arclengthSplining.hpp"
#include "localization/odometry.hpp"
#include "mpc/referenceRollout.hpp"
#define EIGEN_STACK_ALLOCATION_LIMIT 1048576
#include <Eigen/Dense>
#include <cstddef>

//builds the MPC reference trajectory: F+1 target poses (x, y, theta) rolled out along an ALS path
//the rollout itself is mpc_ref::rollout (referenceRollout.hpp), the same code the brain's MPCSerial runs
template<std::size_t F>
class MPCReference {
    public:
        //timing, drivetrain, and reference-gen limits (set once at construction, defaults from mpcConfig.hpp except track width)
        using Params = mpc_ref::Params;

        static constexpr std::size_t r_states = 3; //x, y, theta per stage
        using ZRef = Eigen::Matrix<double, r_states * (F + 1), 1>;

        explicit MPCReference(const Params& params);

        //forget last cycle's reference speed (call on new path)
        void reset() {
            m_v_ref = -1.0;
        }

        //roll out F+1 reference poses from the closest sample idx (idx must be a valid sample index)
        //values are rounded through float to match what the brain sends over serial
        ZRef build(const Pose& currentPose, const ALS_Path& path, std::size_t idx, bool reverse, double omega_L, double omega_R);

    private:
        Params m_params;
        //last cycle's 1st stage reference speed, seeds the next rollout (-1 = none yet)
        double m_v_ref = -1.0;
};

#endif
