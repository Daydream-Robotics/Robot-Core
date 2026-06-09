#include "mpc.hpp"

#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>
#include <algorithm>

template<std::size_t V, std::size_t F>
MPCController<V, F>::Params::Params(
    double wheelRadius,
    double trackWidth,
    double motorA,
    double motorB,
    double samplePeriod,
    double pX,
    double pY,
    double pTheta,
    double qU,
    double q_zero_multiplier,
    double q_final_multiplier,
    double p_final_multiplier,
    double maxVoltage,
    double rBattery,
    double maxDeltaU,
    double minDeltaU,
    double maxFieldX,
    double maxFieldY,
    double maxMotorOmega
)
    : r(wheelRadius),
    L(trackWidth),
    a(motorA),
    b(motorB),
    h(samplePeriod),
    p_x(pX),
    p_y(pY),
    p_theta(pTheta),
    q_u(qU),
    q_zero_multiplier(qZeroMultiplier),
    q_final_multiplier(qFinalMultiplier),
    p_final_multiplier(pFinalMultiplier),
    V_max(maxVoltage),
    R_internal(rBattery),
    delta_u_max(maxDeltaU),
    delta_u_min(minDeltaU),
    x_field_max(maxFieldX),
    y_field_max(maxFieldY),
    omega_motor_max(maxMotorOmega) {}

template<std::size_t V, std::size_t F>
MPCController<V, F>::MPCController(const Params& params)
    : m_params(params)
{
    //fill all matrices with 0s
    m_Ac.setZero();
    m_A.setZero();
    m_Bc.setZero();
    m_B.setZero();
    C.setZero(); 
    C_xy.setZero();
    C_omega.setZero();

    S.setZero();
    
    m_O.setZero();
    m_M.setZero();

    m_W_one.setZero();
    m_W_two.setZero();
    m_W_three.setZero();
    m_W_four.setZero();

    m_Q_i.setZero();
    m_P_i.setZero();

    m_G_delta.setZero();
    m_G_u.setZero();
    m_G_b.setZero();
    m_G_z_xy.setZero();
    m_G_z_omega.setZero();

    m_b_delta.setZero();
    m_b_u.setZero();
    m_b_b.setZero();
    m_b_z_xy.setZero();
    m_b_z_omega.setZero();

    m_b.setZero();

    m_z_xy_max.setZero();
    m_z_omega.setZero();
    
    //initialize C with an I_3 matrix in top left
    C.topLeftCorner(r_states, r_states).setIdentity();
    //initialize b to have the bottom 2 rows be [b 0\\ 0 b]
    m_Bc.bottomRows<2>() = b * Eigen::Matrix2d::Identity();
    //initialize S as [I_m 0 ... 0]
    S.leftCols<m_inputs>().setIdentity();
    //initialize W_1
    for (std::size_t i = 0; i < V; i++) {
        // diagonal block
        m_W_one.template block<m_inputs, m_inputs>(i * m_inputs, i * m_inputs).setIdentity();
        // subdiagonal block
        if (i > 0) {
            m_W_one.template block<m_inputs, m_inputs>(i * m_inputs, (i - 1) * m_inputs) = 
                -Eigen::Matrix<double, m_inputs, m_inputs>::Identity();
        }
    }
    //initialize Q_i
    m_Q_i.diagonal().setConstant(qU);
    //initialize W_2
    m_W_two.template block<m_inputs,m_inputs>(0, 0) = m_params.q_zero_multiplier * m_Q_i;
    for (std::size_t i = 1; i < V-1; i++) {
        m_W_two.template block<m_inputs,m_inputs>(m_inputs * i, m_inputs * i) = m_Q_i;
    }
    m_W_two.template block<m_inputs,m_inputs>(m_inputs*(V - 1), m_inputs*(V - 1)) = m_params.q_final_multiplier * m_Q_i;
    //initialize W_3
    m_W_three.noalias()=m_W_one.transpose() * m_W_two * m_W_one;
    //initiailize P_i
    m_P_i.diagonal()(0)=m_params.p_x;
    m_P_i.diagonal()(1)=m_params.p_y;
    m_P_i.diagonal()(2)=m_params.p_theta;
    //initialize W_4
    for (std::size_t i = 0; i<F-1; i++) {
        m_W_four.template block<n_states, n_states>(n_states * i, n_states * i) = m_P_i;
    }
    m_W_four.template block<n_states, n_states>(n_states*(F-1), n_states*(F-1)) = m_params.p_final_multiplier * m_P_i;
    //initialize G_delta
    m_G_delta.template block <2*V, 2*V> (0,0) = -m_W_one;
    m_G_delta.template block <2*V, 2*V> (2*V,0) = m_W_one;
    //initialize G_u
    m_G_u.template block <2*V, 2*V> (0,0) = -Eigen::Matrix<double, 2*V, 2*V>::Identity();
    m_G_u.template block <2*V, 2*V> (2*V,0) = Eigen::Matrix<double, 2*V, 2*V>::Identity();
    //initialize G_b
    m_G_b=m_G_b;
    //initialize b_Delta
    m_b_delta.topRows<m*V>().setConstant(-m_params.delta_u_min);
    m_b_delta.bottomRows<m*V>().setConstant(m_params.delta_u_max);
    //initialize b_u
    m_b_u.topRows<m*V>().setConstant(m_params.V_max);
    m_b_u.bottomRows<m*V>().setConstant(m_params.V_max);
    //initialize z_xy
    for (std::size_t i = 0; i<F; i++) {
        m_z_xy_max.template segment<2>(2*i) << x_field_max, y_field_max;
    }
    //initialize z_omega
    for (std::size_t i = 0; i<F; i++) {
        m_z_omega_max.template segment<2>(2*i) << omega_motor_max, omega_motor_max;
    }
    //initialize C_xy
    C_xy(0,0) = 1;
    C_xy(1,1) = 1;
    //initialize C_omega
    C_omega(0,3) = 1;
    C_omega(1,4) = 1;
}


template<std::size_t V, std::size_t F>
void MPCController<V, F>::linearize(const Pose& x_hat, double omega_L, double omega_R) {

}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::discretize() {

}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildPredictionMatrices() {

}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::assembleQP(const Eigen::VectorXd& z_ref, const Eigen::VectorXd& x_hat) {

}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::assembleConstraints(double V_batt, const Eigen::Vector2d& u_prev) {

}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildConstraintDelta(const Eigen::Vector2d& u_prev) {

}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildConstraintU() {

}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildConstraintBattery(double V_battery) {

}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildConstraintPosition() {

}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildConstraintHeading() {

}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildConstraintOmega() {

}

template class MPCController<10, 20>;