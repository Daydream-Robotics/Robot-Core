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
    m_C_xy.setZero();
    m_C_omega.setZero();

    S.setZero();
    
    m_O.setZero();
    m_M.setZero();

    m_O_xy.setZero();
    m_O_omega.setZero();
    m_M_xy.setZero();
    m_M_omega.setZero();

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

    z_desired.setZero();
    
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
    m_C_xy(0,0) = 1;
    m_C_xy(1,1) = 1;
    //initialize C_omega
    m_C_omega(0,3) = 1;
    m_C_omega(1,4) = 1;
}

inline double wrapAngle(double angle) {
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}


template<std::size_t V, std::size_t F>
void MPCController<V, F>::linearize(const Pose& x_hat, double omega_L, double omega_R) {
    this->x_hat.x = pose.x;
    this->x_hat.y = pose.y;
    this->x_hat.theta = pose.theta;
    this->x_hat.omega_L = omega_L;
    this->x_hat.omega_R = omega_R;
    //store cos and sin
    double cos_theta = cos(x_hat.theta);
    double sin_theta = sin(x_hat.theta);
    //initailize A^c_k
    m_Ac = << 
    0, 0, -R_TWO*(omega_L+omega_R)*sin_theta, R_TWO*cos_theta, R_TWO*cos_theta,
    0,0,   R_TWO*(omega_L+omega_R)*cos_theta, R_2*sin_theta, R_TWO*sin_theta,
    0,0,0, -R_L, R_L,
    0,0,0, -m_params.a, 0,
    0,0,0,0, -m_params.a;
}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::discretize() {
    //declares a matrix to hold exp version of continuous A&B matrices
    Eigen::Matrix<double, n_states + m_inputs, n_states + m_inputs> m_exp;
    M.setZero();
    //puts A^C_K and B^c in exp matrix
    M.template block<n_states, n_states>(0, 0) = m_Ac;
    M.template block<n_states, m_inputs>(0, n_states) = m_Bc;
    //multiplies exp matrix by sample period
    m_exp *= m_params.h;
    // exponentiates matrix
    auto Md = M.exp();
    // extract A_k
    m_A = Md.template block<n_states, n_states>(0, 0);
    // extract B_k
    m_B = Md.template block<n_states, m_inputs>(0, n_states);
}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildPredictionMatrices() {
    //makes matrix to hold A to a power
    Eigen::Matrix<double, n_states, n_states> m_A_power = m_A;
    //makes O matrix
    for (std::size_t i = 0; i<F; i++){
        m_O.template block<r_states, n_states> (i*r_states, 0) = m_C * m_A_power;
        m_A_power *= m_A 
    }
    //initialize M
    for (std::size_t i = 0; i < F; i++) {

        for (std::size_t j = 0; j < V; j++) {
            // upper triangular zero region
            if (j > i) {
                continue;
            }
            Eigen::Matrix<double, n_states, n_states> A_term = Eigen::Matrix<double, n_states, n_states>::Identity();
            //normal region
            if (i < V || j < V - 1) {
                for (std::size_t k = 0; k < (i - j); k++)
                    A_term *= m_A;
                m_M.template block<r_states, m_inputs>(i * r_states, j * m_inputs) = C * A_term * m_B;
            }
            //held control region
            else {
                Eigen::Matrix<double, n_states, n_states> A_bar = Eigen::Matrix<double, n_states, n_states>::Zero();
                Eigen::Matrix<double, n_states, n_states> A_sum = Eigen::Matrix<double, n_states, n_states>::Identity();
                for (std::size_t k = 0; k <= (i - V + 1); k++) {
                    if (k > 0) {
                        A_sum *= m_A;
                    }
                    A_bar += A_sum;
                }
                for (std::size_t k = 0; k < (V - 1 - j); k++) {
                    A_term *= m_A;
                }
                m_M.template block<r_states, m_inputs>(i * r_states, j * m_inputs) = C * A_term * A_bar * m_B;
            }
        }
    }
    m_O_xy=C_xy*O;
    m_O_omega=C_omega*O;
    m_M_xy=C_xy*M;
    m_M_omega=C_omega*M;
}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::assembleConstraints(double V_batt, const Eigen::Vector2d& u_prev) {
    buildConstraintDelta(u_prev);
    buildConstraintU();
    buildConstraintBattery();
    buildConstraintPosition();
    buildConstraintOmega();
}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildConstraintDelta(const Eigen::Vector2d& u_prev) {
    //add prev u to b_Dleta
    m_b_b.template segment<m_inputs>(0) += u_prev;
    m_b_b.template segment<m_inputs>(m_inputs * V) -= u_prev;
}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildConstraintBattery(double V_battery, double I_total) {
    //initialize max input due to battery sag
    double u_b_max = V_battery - m_params.R_internal*I_total
    //initialize b_battery 
    m_b_delta.topRows<m*V>().setConstant(u_b_max);
    m_b_delta.bottomRows<m*V>().setConstant(u_b_max);

}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildConstraintPosition() {
    //initialize b_z,xy
    m_b_z_xy.topRows(2 * F) = z_max_xy+ m_O_xy*x_hat;    
    m_b_z_xy.bottomRows(2 * F) = z_max_xy - m_O_xy*x_hat;
}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildConstraintOmega() {
    //initialize b_z,omega
    m_b_z_omega.topRows(2 * F) = z_max_omega+ m_O_omega*x_hat;    
    m_b_z_omega.bottomRows(2 * F) = z_max_xy - m_O_omega*x_hat;
}

template<std::size_t V, std::size_t F>
InterpSample sampleAtArcLength(const std::vector<Sample>& samples, double sQuery) {
    InterpSample result;
    if (samples.empty()) {
        return result;
    }
    if (sQuery <= samples.front().s) {
        result.x = samples.front().x;
        result.y = samples.front().y;
        result.theta = samples.front().theta;
        result.v = samples.front().v;
        return result;
    }
    if (sQuery >= samples.back().s) {
        result.x = samples.back().x;
        result.y = samples.back().y;
        result.theta = samples.back().theta;
        result.v = samples.back().v;
        return result;
    }

    std::size_t left = 0;
    std::size_t right = samples.size()-1;

    while (left+1 < right) {
        std::size_t mid = left + (right-left)/2;
        if (samples[mid].s <= sQuery) {
            left = mid;
        }
        else {
            right = mid;
        }
    }
    const Sample& a = samples[left];
    const Sample& b = samples[right];

    double ds = b.s - a.s;
    double alpha = (std::abs(ds) < 1e-12) ? 0.0 : (sQuery - a.s) /ds
    double dtheta = wrapAngle(b.heading-a.heading);

    result.x = a.x + alpha * (b.x-a.x);
    result.y = a.y + alpha * (b.y-a.y);
    result.heading = wrapAngle(a.heading + alpha * dtheta);
    result.v = a.v + alpha * (b.v-a.v)

    return result;
}

template<std::size_t V, std::size_t F>
void buildZDesired(const ALS_Path& als_path, std::size_t closestSampleIdx) {
    const std::vector<Sample>& samples = als_path.getSamples();

    if (samples.empty() || !als_path.isValid()) {
        return;
    }
    if (closestSampleIdx >= samples.size()) {
        closestSampleIdx = samples.size()-1;
    }

    double s_prev = samples[closestSampleIdx].s;

    for (std::size_t i = 0; i < F; i++) {
        InterpSample curr = sampleAtArcLength(samples, s_prev);
        double v_path = std::max(curr.v, 0.0);
        s_prev += v_path * m_params.h;

        s_prev = std::min(s_prev, als_path.getTotalLength());

        InterpSample ref = sampleAtArcLength(samples, s_prev);

        const int base = i * r_states;
        z_desired(base+0) = ref.x;
        z_desired(base+1) = ref.y;
        z_desired(base+2) = ref.theta;
    }
}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::assembleQP() {

}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::solveQP() {

}

template class MPCController<40, 40>;