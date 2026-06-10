#ifndef _MPC_HPP_
#define _MPC_HPP_

#include "motionController.hpp"
#include "odometry.hpp"
#include "arclengthSplining.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <cstddef>

template<std::size_t V, std::size_t F>
class MPCController : public MotionController {
    public:
        struct Params {
            double r; //wheel radius (in)
            double L; //track width (in)
            double a; //motor time-constant
            double b; //motor gain constant
            double h; //sampling period (s)
            double p_x; //output weight for x
            double p_y; //output weight for y
            double p_theta; //output weight for theta
            double q_u; //input penalty
            double q_zero_multiplier; //multiplier for first Q_i
            double q_final_multiplier; //multiplier for last Q_i
            double p_final_multiplier; //multiplier for last P_i
            double V_max; //voltage max
            double R_internal; //internal resistance of battery
            double delta_u_max; //max positive change in voltage between steps
            double delta_u_min; //max negative change in voltage between steps
            double x_field_max; //max allowed x position on field
            double y_field_max; //max allowed y position on field
            double omega_motor_max; //max allowed rpm

            Params(
                double wheelRadius,
                double trackWidth,
                double motorA,
                double motorB,
                double samplePeriod,
                double pX,
                double pY,
                double pTheta,
                double qU,
                double qZeroMultiplier,
                double qFinalMultiplier,
                double pFinalMultiplier,
                double maxVoltage,
                double rBattery,
                double maxDeltaU,
                double minDeltaU,
                double maxFieldX,
                double maxFieldY,
                double maxMotorOmega
            );
        };

        explicit MPCController(const Params& params);
        ~MPCController() override = default;

        WheelVelocities compute(const Pose& currentPose, const ALS_Path& als_path, std::size_t& closestSampleIdx) override;
        void reset() override;

        private:

            struct State {
                // x-position of bot (inches)
                double x;

                // y-position of bot (inches)
                double y;

                // Heading of bot (rads)
                double theta;

                double omega_L;

                double omega_R;
            };

            static constexpr std::size_t n_states = 5;
            static constexpr std::size_t m_inputs = 2;
            static constexpr std::size_t r_states = 3;
            static constexpr std::size_t num_constr = 6;

            const double R_L = r/L;
            const double R_TWO = r/2;

            double battery_voltage;
            double total_current; 

            State x_hat;
            
            Params m_params;

            Eigen::Matrix<double, n_states, n_states> m_Ac;
            Eigen::Matrix<double, n_states, n_states> m_A;
            Eigen::Matrix<double, n_states, m_inputs> m_Bc;
            Eigen::Matrix<double, n_states, m_inputs> m_B;
            Eigen::Matrix<double, n_states, n_states> C;
            Eigen::Matrix<double, 2, n_states> m_C_xy;
            Eigen::Matrix<double, 2, n_states> m_C_omega;

            Eigen::Matrix<double, 2, 2*V> S;

            Eigen::Matrix<double, r_states*F, n_states> m_O;
            Eigen::Matrix<double, r_states*F, m_inputs*V> m_M;

            Eigen::Matrix<double, 2*F, n_states> m_O_xy;
            Eigen::Matrix<double, 2*F, n_states> m_O_omega;
            Eigen::Matrix<double, 2*F, n_states> m_M_xy;
            Eigen::Matrix<double, 2*F, n_states> m_M_omega;

            Eigen::Matrix<double, m_inputs*V, m_inputs*V> m_W_one;
            Eigen::Matrix<double, m_inputs*V, m_inputs*V> m_W_two;
            Eigen::Matrix<double, m_inputs*V, m_inputs*V> m_W_three;
            Eigen::Matrix<double, n_states*F, n_states*F> m_W_four;

            Eigen::Matrix<double, m_inputs, m_inputs> m_Q_i;
            Eigen::Matrix<double, n_states, n_states> m_P_i;


            Eigen::Matrix<double, 2*m_inputs*V, m_inputs*V> m_G_delta;
            Eigen::Matrix<double, 2*m_inputs*V, m_inputs*V> m_G_u;
            Eigen::Matrix<double, 2*m_inputs*V, m_inputs*V> m_G_b;
            Eigen::Matrix<double, 4*F, m_inputs*V> m_G_z_xy;
            Eigen::Matrix<double, 4*F, m_inputs*V> m_G_z_omega;

            Eigen::SparseMatrix<double> m_G;

            Eigen::Matrix<double, 2*m_inputs*V, 1> m_b_delta;
            Eigen::Matrix<double, 2*m_inputs*V, 1> m_b_u;
            Eigen::Matrix<double, 2*m_inputs*V, 1> m_b_b;
            Eigen::Matrix<double, 4*F, 1> m_b_z_xy;
            Eigen::Matrix<double, 4*F, 1> m_b_z_omega;

            Eigen::Matrix<double, 6*m_inputs*V + 8*F, 1> m_b;

            Eigen::Matrix<double, 2*F, 1> m_z_xy_max;
            Eigen::Matrix<double, 2*F, 1> m_z_omega_max;


            void linearize(const Pose& x_hat, double omega_L, double omega_R);
            void discretize();
            void buildPredictionMatrices();
            Eigen::Vector<double,r_states*F, 1> buildZDesired(const ALS_Path& als_path, std::size_t closestSampleIdx);
            // Assemble all into m_G and m_b
            void assembleConstraints(double V_batt, const Eigen::Vector2d& u_prev);
            // Helper methods to build each constraint type
            void buildConstraintDelta(const Eigen::Vector2d& u_prev); 
            void buildConstraintU();        
            void buildConstraintBattery(double V_battery, double I_total);  
            void buildConstraintPosition(); 
            void buildConstraintOmega();
            void assembleQP();
            void solveQP();
            
};
#endif