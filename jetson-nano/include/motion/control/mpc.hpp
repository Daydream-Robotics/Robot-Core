#ifndef _MPC_HPP_
#define _MPC_HPP_

#include "motion/control/motionController.hpp"
#include "motion/control/mpcReference.hpp"
#include "localization/odometry.hpp"
#include "motion/pathing/arclengthSplining.hpp"
#include "config/constants.h"
#include "config/robotDrive.hpp"
#include "mpc/drivetrain.hpp"
#include "osqp.h"

#include <cstddef>
#include <array>
#include <vector>

#define EIGEN_STACK_ALLOCATION_LIMIT 1048576
#include <Eigen/Dense>
#include <Eigen/Sparse>

//LTV MPC controller: Drive = drivetrain (kinematics::DiffDrive, kinematics::XDrive, kinematics::HDrive), V = control horizon, F = prediction horizon
//pose frame: +x forward, +y left; theta = 0 along +y, pi/2 along +x (see Pose)
//model: body twist (vx, vy, w) = K*omega from the group speeds (see mpc/drivetrain.hpp), then in the pose frame
//x_dot = vx*sin(theta) - vy*cos(theta), y_dot = vx*cos(theta) + vy*sin(theta), theta_dot = -w
//diff drive reduces to v = r*(omega_L + omega_R)/2, x_dot = v*sin(theta), y_dot = v*cos(theta), theta_dot = r*(omega_L - omega_R)/L
template<class Drive, std::size_t V, std::size_t F>
class MPCController : public MotionController {
    public:
        //dimensions
        static constexpr std::size_t m_inputs = Drive::N_GROUPS; //inputs: one voltage per motor group
        static constexpr std::size_t n_states = 3 + m_inputs; //state: x, y, theta, group speeds
        static constexpr std::size_t r_states = 3; //tracked outputs: x, y, theta
        static constexpr std::size_t num_constr = 5; //number of constraint types
        static constexpr std::size_t n_constr = 6 * m_inputs * V + 4 * F + 2 * m_inputs * F; //rows of the stacked constraint matrix

        static_assert(m_inputs <= kinematics::MAX_MOTOR_GROUPS, "drive has more groups than kinematics::MAX_MOTOR_GROUPS");

        using VecM = Eigen::Matrix<double, m_inputs, 1>;
        using WheelArray = std::array<double, m_inputs>;

        //all model, cost, and constraint parameters (set once at construction)
        struct Params {
            kinematics::DriveGeometry geom; //drivetrain geometry, read by Drive::G and Drive::K
            WheelArray a{}; //motor time-constant per group
            WheelArray b{}; //motor gain constant per group
            WheelArray omega_motor_max{}; //max allowed group speed (rad/s)
            WheelArray delta_u_max{}; //max positive change in voltage between steps per group
            WheelArray delta_u_min{}; //max negative change in voltage between steps per group
            WheelArray V_applied{}; //starting voltage per group
            double h = 0.0; //sampling period (s)
            double p_x = 0.0; //output weight for x
            double p_y = 0.0; //output weight for y
            double p_theta = 0.0; //output weight for theta
            double q_u = 0.0; //input penalty
            double q_zero_multiplier = 1.0; //multiplier for first Q_i
            double q_final_multiplier = 1.0; //multiplier for last Q_i
            double p_final_multiplier = 1.0; //multiplier for last P_i
            double V_max = 0.0; //voltage max
            double x_field_max = 0.0; //max allowed x position on field
            double y_field_max = 0.0; //max allowed y position on field
            double command_delay = 0.0; //s from solving to the command being applied, for predict()
            HeadingMode heading = HeadingMode::TANGENT; //reference heading along the path, X and H drives can use HOLD

            //from the [MPC] and [ROBOT DIMS] constants in config/constants.h, starting at 0 V
            //every group gets the same motor constants, an X or H drive still has to set geom.rho or geom.strafeRadius
            Params();
        };

        //throws std::invalid_argument if params.geom is missing a value this drive needs
        explicit MPCController(const Params& params = Params());
        //frees the OSQP workspace
        ~MPCController() override;
        //full reset to just-constructed state for a new path (u_prev, OSQP warm start, reference speed)
        //also call it after the drive was stopped from outside, so the delta-u limit restarts from 0 V
        void reset() override;

        //builds the reference from the path and solves. returns INPUT_VOLTAGE in mV per group (0 mV if the solve fails)
        //drive.omega and the command use Drive's group order
        DriveCommand compute(const Pose& currentPose, const DriveState& drive, const ALS_Path& als_path, std::size_t& closestSampleIdx, PathFlag flag) override;

        //moves the wheel speeds forward by their age plus command_delay, and the pose by command_delay,
        //with the motor model and the voltage this controller last sent (what the brain is applying)
        void predict(Pose& pose, DriveState& drive) const override;

        //per-group version of predict(): omega is advanced in place, age is how old omega is (s)
        void predictState(Pose& pose, VecM& omega, double age, double batteryVolts) const;

        //full MPC solve from state + reference trajectory, returns optimal voltages per group (V). throws if the solve fails
        //omega: group speeds (rad/s). V_battery: battery voltage measured under load (V), caps the input voltage
        VecM compute(const Pose& currentPose, const Eigen::Matrix<double, r_states * (F + 1), 1>& z_desired, const VecM& omega, double V_battery);

        //sync m_u_prev with the voltage actually applied (for delta-u constraint)
        //call every cycle before compute(), after any reset()
        void notifyAppliedVoltage(const VecM& u) {
                m_u_prev = u;
        }

        const kinematics::DriveGeometry& geometry() const { return m_params.geom; }

        private:

             //stuct to own the arrays behind an OSQP CSC matrix (keeps pointers valid)
            struct CscStorage {
                std::vector<OSQPFloat> values;
                std::vector<OSQPInt>   row_idx;
                std::vector<OSQPInt>   col_ptr;
                OSQPCscMatrix mat;
            };

            Params m_params;
            //reference trajectory along the path, built with the same model params
            MPCReference<Drive, F> m_reference;

            //drivetrain kinematics: group speeds = G * body twist, body twist = K * group speeds
            Eigen::Matrix<double, m_inputs, 3> m_G_kin;
            Eigen::Matrix<double, 3, m_inputs> m_K_kin;
            //per-group motor constants as vectors
            VecM m_a_vec;
            VecM m_b_vec;

            //1st stage voltages from the latest solve
            VecM m_u_opt;

            //matrices for LTV model: x_{k+1} = A_k*x_k + B_k*u_k + d_k
            std::array<Eigen::Matrix<double, n_states, n_states>, F> m_A_k;
            std::array<Eigen::Matrix<double, n_states, m_inputs>, F> m_B_k;
            std::array<Eigen::Matrix<double, n_states, 1>, F> m_d_k;
            //matrices for ref traject
            Eigen::Matrix<double, r_states*(F+1), 1> m_z_ref;
            Eigen::Matrix<double, n_states, F+1> m_x_ref;
            Eigen::Matrix<double, m_inputs, F> m_u_ref;
            //matrices for stacked affine response terms
            Eigen::Matrix<double, r_states*F, 1> m_D_z;
            Eigen::Matrix<double, 2*F, 1> m_D_xy;
            Eigen::Matrix<double, m_inputs*F, 1> m_D_omega;

            //matrices for single-model system matrices
            Eigen::Matrix<double, n_states, n_states> m_Ac;
            Eigen::Matrix<double, n_states, n_states> m_A;
            Eigen::Matrix<double, n_states, m_inputs> m_Bc;
            Eigen::Matrix<double, n_states, m_inputs> m_B;
            //matrices for output maps
            Eigen::Matrix<double, r_states, n_states> m_C;
            Eigen::Matrix<double, 2, n_states> m_C_xy;
            Eigen::Matrix<double, m_inputs, n_states> m_C_omega;

            //prediction matrices
            Eigen::Matrix<double, r_states*F, n_states> m_O;
            Eigen::Matrix<double, r_states*F, m_inputs*V> m_M;
            //prediction matrices for position and wheel speed outputs (for constr)
            Eigen::Matrix<double, 2*F, n_states> m_O_xy;
            Eigen::Matrix<double, m_inputs*F, n_states> m_O_omega;
            Eigen::Matrix<double, 2*F, m_inputs*V> m_M_xy;
            Eigen::Matrix<double, m_inputs*F, m_inputs*V> m_M_omega;

            //current state estimate
            Eigen::Matrix<double, n_states, 1> m_x_hat;

            //cost weights matrices
            Eigen::Matrix<double, m_inputs*V, m_inputs*V> m_W_one;
            Eigen::Matrix<double, m_inputs*V, m_inputs*V> m_W_two;
            Eigen::Matrix<double, m_inputs*V, m_inputs*V> m_W_three;
            Eigen::Matrix<double, r_states*F, r_states*F> m_W_four;

            //weight block matrices (Q for input changes and P for output errors)
            Eigen::Matrix<double, m_inputs, m_inputs> m_Q_i;
            Eigen::Matrix<double, n_states, n_states> m_P_i;

            //constraint matirces
            Eigen::Matrix<double, 2*m_inputs*V, m_inputs*V> m_G_delta;
            Eigen::Matrix<double, 2*m_inputs*V, m_inputs*V> m_G_u;
            Eigen::Matrix<double, 2*m_inputs*V, m_inputs*V> m_G_b;
            Eigen::Matrix<double, 4*F, m_inputs*V> m_G_z_xy;
            Eigen::Matrix<double, 2*m_inputs*F, m_inputs*V> m_G_z_omega;
            //stacked constraint matrices
            Eigen::Matrix<double, n_constr, m_inputs*V> m_G;
            //matrices for right hand side of constraints
            Eigen::Matrix<double, 2*m_inputs*V, 1> m_b_delta;
            Eigen::Matrix<double, 2*m_inputs*V, 1> m_b_u;
            Eigen::Matrix<double, 2*m_inputs*V, 1> m_b_b;
            Eigen::Matrix<double, 4*F, 1> m_b_z_xy;
            Eigen::Matrix<double, 2*m_inputs*F, 1> m_b_z_omega;
            //all right hand side matrices stacked into one b matrix
            Eigen::Matrix<double, n_constr, 1> m_b;

            //output limits per prediction step in matrices
            Eigen::Matrix<double, 2*F, 1> m_z_xy_max;
            Eigen::Matrix<double, m_inputs*F, 1> m_z_omega_max;

            //reference poses and tracking error matrices
            Eigen::Matrix<double, r_states * (F + 1), 1> m_z_desired;
            Eigen::Matrix<double, r_states*F, 1> m_s;

            //condensed QP matrices
            Eigen::Matrix<double, m_inputs*V, m_inputs*V> m_P;
            Eigen::Matrix<double, m_inputs*V, 1> m_q;

            //matrix for last applied voltages
            Eigen::Matrix<double, m_inputs, 1> m_u_prev;

            //OSQP workspace and persistent CSC/vector buffers which is reused across solves
            OSQPSolver* m_solver = nullptr;
            CscStorage  m_P_csc;
            CscStorage  m_A_full_csc;
            std::vector<OSQPFloat> m_q_v, m_l_v, m_u_v;
            //shifted previous solution used to warm start the next solve (per instance)
            std::vector<OSQPFloat> m_x_ws;


            //linearize and discretize about each ref state (per-stage LTV models)
            void buildStageModels();
            //reconstruct full ref states and feedforward inputs from ref poses
            void buildReferenceStates();
            //build O, M, D pred. matrices from the per-stage models
            void buildPredictionMatrices();

            //assemble all into m_G and m_b
            void assembleConstraints(double V_batt, const Eigen::Matrix<double, m_inputs, 1> u_prev);
            //fill per-group delta-u bounds for every control step
            void fillDeltaBounds();
            //helper methods to build each constraint type
            void buildConstraintDelta(const Eigen::Matrix<double, m_inputs, 1> u_prev);
            void buildConstraintBattery(double V_battery);
            void buildConstraintPosition();
            void buildConstraintOmega();

            //form the condensed QP cost (P, q)
            void assembleQP();

            //dense-pattern CSC helpers which allocates once then rewrites values each solve
            static void initCscDense(CscStorage& out, int rows, int cols, bool upperOnly);
            template<typename Derived>
            static void fillCscDense(CscStorage& out, const Eigen::MatrixBase<Derived>& M, bool upperOnly);

            //solve the QP cost with OSQP
            void solveQP();
};

//the MPC for this robot's drivetrain (RobotDrive in config/robotDrive.hpp) with the horizons in config/constants.h
using MPC = MPCController<RobotDrive, MPC_CONTROL_HORIZON, MPC_PREDICTION_HORIZON>;

#endif

