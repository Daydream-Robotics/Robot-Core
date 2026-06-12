#ifndef _MPC_HPP_
#define _MPC_HPP_

#include "motionController.hpp"
#include "odometry.hpp"
#include "arclengthSplining.hpp"
#include "serial_protocol.hpp"
#include "osqp/osqp.h"
#define EIGEN_STACK_ALLOCATION_LIMIT 1048576 
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
            double V_left_applied; //starting Voltage left
            double V_right_applied; //starting Voltage right

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
                double maxMotorOmega, 
                double voltageLeft,
                double voltageRight
            );
        };
        static constexpr std::size_t n_states = 5;
        static constexpr std::size_t m_inputs = 2;
        static constexpr std::size_t r_states = 3;
        static constexpr std::size_t num_constr = 5;

        explicit MPCController(const Params& params);
        ~MPCController() override = default;
        void reset() override;

        WheelVelocities compute(const Pose& currentPose, const ALS_Path& als_path, std::size_t& closestSampleIdx) 
            override ;
        
        WheelVelocities compute(const Pose& currentPose, Eigen::Matrix<double, r_states*F, 1> z_desired, double omega_L, double omega_R, double V_battery, double I_total);

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
            #pragma pack(push, 1)
            struct MPCUpdatePacket {
                float pose_x;
                float pose_y;
                float pose_theta;
                float omega_L;
                float omega_R;
                float V_battery;
                float I_total;
                float z_desired[F*3];
            };
            #pragma pack(pop)
            #pragma pack(push, 1)
            struct MPCControlPacket {
                float V_left;
                float V_right;
            };
            #pragma pack(pop)

            struct InterpSample {
                double x       = 0.0;
                double y       = 0.0;
                double theta = 0.0;
                double v       = 0.0;
            };

            Params m_params;
            State x_hat;

            const double R_L = m_params.r/m_params.L;
            const double R_TWO = m_params.r/2;

            double battery_voltage;
            double total_current; 

            double m_u_left;
            double m_u_right;

            double u_prev = 0;


            Eigen::Matrix<double, n_states, n_states> m_Ac;
            Eigen::Matrix<double, n_states, n_states> m_A;
            Eigen::Matrix<double, n_states, m_inputs> m_Bc;
            Eigen::Matrix<double, n_states, m_inputs> m_B;
            Eigen::Matrix<double, r_states, n_states> m_C;
            Eigen::Matrix<double, 2, n_states> m_C_xy;  
            Eigen::Matrix<double, 2, n_states> m_C_omega; 

            Eigen::Matrix<double, r_states*F, n_states> m_O;
            Eigen::Matrix<double, r_states*F, m_inputs*V> m_M;

            Eigen::Matrix<double, 2*F, n_states> m_O_xy;
            Eigen::Matrix<double, 2*F, n_states> m_O_omega;
            Eigen::Matrix<double, 2*F, m_inputs*V> m_M_xy;
            Eigen::Matrix<double, 2*F, m_inputs*V> m_M_omega;

            Eigen::Matrix<double, n_states, 1> m_x_hat;

            Eigen::Matrix<double, m_inputs*V, m_inputs*V> m_W_one;
            Eigen::Matrix<double, m_inputs*V, m_inputs*V> m_W_two;
            Eigen::Matrix<double, m_inputs*V, m_inputs*V> m_W_three;
            Eigen::Matrix<double, r_states*F, r_states*F> m_W_four;

            Eigen::Matrix<double, m_inputs, m_inputs> m_Q_i;
            Eigen::Matrix<double, n_states, n_states> m_P_i;


            Eigen::Matrix<double, 2*m_inputs*V, m_inputs*V> m_G_delta;
            Eigen::Matrix<double, 2*m_inputs*V, m_inputs*V> m_G_u;
            Eigen::Matrix<double, 2*m_inputs*V, m_inputs*V> m_G_b;
            Eigen::Matrix<double, 4*F, m_inputs*V> m_G_z_xy;
            Eigen::Matrix<double, 4*F, m_inputs*V> m_G_z_omega;

            Eigen::Matrix<double, 6*m_inputs*V + 8*F, m_inputs*V> m_G;

            Eigen::Matrix<double, 2*m_inputs*V, 1> m_b_delta;
            Eigen::Matrix<double, 2*m_inputs*V, 1> m_b_u;
            Eigen::Matrix<double, 2*m_inputs*V, 1> m_b_b;
            Eigen::Matrix<double, 4*F, 1> m_b_z_xy;
            Eigen::Matrix<double, 4*F, 1> m_b_z_omega;

            Eigen::Matrix<double, 6*m_inputs*V + 8*F, 1> m_b;

            Eigen::Matrix<double, 2*F, 1> m_z_xy_max;
            Eigen::Matrix<double, 2*F, 1> m_z_omega_max;

            Eigen::Matrix<double, r_states*F, 1> m_z_desired;
            Eigen::Matrix<double, r_states*F, 1> m_s;

            Eigen::Matrix<double, m_inputs*V, m_inputs*V> m_P;
            Eigen::Matrix<double, m_inputs*V, 1> m_q;

            Eigen::Matrix<double, m_inputs, 1> m_u_prev;


            void linearize(const Pose& x_hat, double omega_L, double omega_R);
            void discretize();
            void buildPredictionMatrices();
            // Assemble all into m_G and m_b
            void assembleConstraints(double V_batt, double I_total, const Eigen::Matrix<double, m_inputs, 1> u_prev);
            // Helper methods to build each constraint type
            void buildConstraintDelta(const Eigen::Matrix<double, m_inputs, 1> u_prev); 
            void buildConstraintU();        
            void buildConstraintBattery(double V_battery, double I_total);  
            void buildConstraintPosition(); 
            void buildConstraintOmega();
            InterpSample sampleAtArcLength(const std::vector<Sample>& samples, double sQuery);
            Eigen::Matrix<double,r_states*F, 1> buildZDesired(const ALS_Path& als_path, std::size_t closestSampleIdx);
            void buildZDesiredFromPacket(const MPCUpdatePacket& packet);
            void assembleQP();
            void solveQP();
            static Eigen::Matrix<double, r_states * F, 1> unpackZDesired(const float* z_raw);
            static void MPCControl(SerialProtocol& serial, MPCController& mpc);
            
};
#endif