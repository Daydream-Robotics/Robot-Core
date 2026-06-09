#ifndef _MPC_HPP_
#define _MPC_HPP_

#include "motionController.hpp"
#include "odometry.hpp"
#include "arclengthSplining.hpp"

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

class MPCController : public MotionController {
    public:
        struct Params {
            double r; //wheel radius (in)
            double L; //track width (in)
            double a; //motor time-constant
            double b; //motor gain constant
            double h; //sampling period (s)
            std::size_t f; //prediction horizon (steps)
            std::size_t v; //control horizon (steps)
            double p_x; //output weight for x
            double p_y; //output weight for y
            double p_theta; //output weight for theta
            double q_u; //input penalty
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
                std::size_t predictionHorizon,
                std::size_t controlHorizon,
                double pX,
                double pY,
                double pTheta,
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
            static constexpr std::size_t n_states = 5;
            static constexpr std::size_t m_inputs = 2;
            static constexpr std::size_t r_states = 3;

            Params m_params;

            Eigen::Matrix<double, n_states, n_states> m_Ac;
            Eigen::Matrix<double, n_states, n_states> m_A;
            Eigen::Matrix<double, n_states, m_inputs> m_Bc;
            Eigen::Matrix<double, n_states, m_inputs> m_B;

            Eigen::MatrixXd m_O;
            Eigen::MatrixXd m_M;

            Eigen::MatrixXd m_W_one;
            Eigen::MatrixXd m_W_two;
            Eigen::MatrixXd m_W_three;
            Eigen::MatrixXd m_W_four;

            Eigen::MatrixXd m_G_delta;
            Eigen::MatrixXd m_G_u;
            Eigen::MatrixXd m_G_b;
            Eigen::MatrixXd m_G_z_xy;
            Eigen::MatrixXd m_G_z_theta;
            Eigen::MatrixXd m_G_z_omega;

            Eigen::SparseMatrix<double> m_G;

            Eigen::VectorXd m_b_delta;
            Eigen::VectorXd m_b_u;
            Eigen::VectorXd m_b_b;
            Eigen::VectorXd m_b_z_xy;
            Eigen::VectorXd m_b_z_theta;
            Eigen::VectorXd m_b_z_omega;

            Eigen::VectorXd m_b;

            
            void linearize(const Pose& x_hat, double omega_L, double omega_R);
            void discretize();
            void buildPredictionMatrices();
            void assembleQP(const Eigen::VectorXd& z_ref, const Eigen::VectorXd& x_hat);
            // Assemble all into m_G and m_b
            void assembleConstraints(double V_batt, const Eigen::Vector2d& u_prev);
            // Helper methods to build each constraint type
            void buildConstraintDelta(const Eigen::Vector2d& u_prev); 
            void buildConstraintU();        
            void buildConstraintBattery(double V_battery);  
            void buildConstraintPosition(); 
            void buildConstraintHeading();
            void buildConstraintOmega();
            
};
#endif