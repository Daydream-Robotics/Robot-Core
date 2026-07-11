#include <iomanip>
#include <iostream>
#include "mpc.hpp"
#define EIGEN_STACK_ALLOCATION_LIMIT 1048576 
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>
#include <algorithm>
#include <fstream>

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
    omega_motor_max(maxMotorOmega),
    V_left_applied(voltageLeft),
    V_right_applied(voltageRight) {}


//constructor
template<std::size_t V, std::size_t F>
MPCController<V, F>::MPCController(const Params& params)
    : m_params(params)
{
    //fill all matrices with 0s
    m_Ac.setZero();
    m_A.setZero();
    m_Bc.setZero();
    m_B.setZero();
    m_C.setZero(); 
    m_C_xy.setZero();
    m_C_omega.setZero();
    
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
    m_z_omega_max.setZero();

    m_z_desired.setZero();
    m_s.setZero();

    m_P.setZero();
    m_q.setZero();
    
    //initialize C with an I_3 matrix in top left
    m_C << 1, 0, 0, 0, 0,
       0, 1, 0, 0, 0,
       0, 0, 1, 0, 0;
    //initialize b to have the bottom 2 rows be [b 0\\ 0 b]
    m_Bc.bottomRows<2>() = m_params.b * Eigen::Matrix2d::Identity();
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
    m_Q_i.diagonal().setConstant(m_params.q_u);
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
        m_W_four.template block<r_states, r_states>(r_states * i, r_states * i) = m_P_i.topLeftCorner<r_states, r_states>();
    }
    m_W_four.template block<r_states, r_states>(r_states*(F-1), r_states*(F-1)) = m_params.p_final_multiplier * m_P_i.topLeftCorner<r_states, r_states>();
    //initialize G_delta
    m_G_delta.template block <2*V, 2*V> (0,0) = -m_W_one;
    m_G_delta.template block <2*V, 2*V> (2*V,0) = m_W_one;
    //initialize G_u
    m_G_u.template block <2*V, 2*V> (0,0) = -Eigen::Matrix<double, 2*V, 2*V>::Identity();
    m_G_u.template block <2*V, 2*V> (2*V,0) = Eigen::Matrix<double, 2*V, 2*V>::Identity();
    //initialize G_b
    m_G_b=m_G_u;
    //initialize b_Delta
    m_b_delta.template topRows<m_inputs*V>().setConstant(-m_params.delta_u_min);
    m_b_delta.template bottomRows<m_inputs*V>().setConstant(m_params.delta_u_max);
    //initialize b_u
    m_b_u.template topRows<m_inputs*V>().setConstant(m_params.V_max);
    m_b_u.template bottomRows<m_inputs*V>().setConstant(m_params.V_max);
    //initialize z_xy
    for (std::size_t i = 0; i<F; i++) {
        m_z_xy_max.template segment<2>(2*i) << m_params.x_field_max, m_params.y_field_max;
    }
    //initialize z_omega
    for (std::size_t i = 0; i<F; i++) {
        m_z_omega_max.template segment<2>(2*i) << m_params.omega_motor_max, m_params.omega_motor_max;
    }
    //initialize C_xy
    m_C_xy << 1, 0, 0, 0, 0,
          0, 1, 0, 0, 0;
    //initialize C_omega
    m_C_omega << 0, 0, 0, 1, 0,
             0, 0, 0, 0, 1;
    //initialize u_prev
    m_u_prev << m_params.V_left_applied, m_params.V_right_applied;
}

//wrap angle to (-pi, pi]
inline double wrapAngle(double angle) {
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}


//linearize continuous model
template<std::size_t V, std::size_t F>
void MPCController<V, F>::linearize(const Pose& x_hat, double omega_L, double omega_R) {
    //pack state vector for QP
    m_x_hat <<
        x_hat.x,
        x_hat.y,
        x_hat.theta,
        omega_L,
        omega_R;
    //store cos and sin
    double cos_theta = cos(x_hat.theta);
    double sin_theta = sin(x_hat.theta);
    //initailize A^c_k
    constexpr double OMEGA_SUM_MIN = 20.0;   // e.g. 2 × 15 rad/s
    double omega_sum = omega_L + omega_R;
    if (std::abs(omega_sum) < OMEGA_SUM_MIN) {
        omega_sum = (omega_sum >= 0.0) ? OMEGA_SUM_MIN : -OMEGA_SUM_MIN;
    }
    m_Ac << 
    0, 0, -R_TWO*omega_sum*sin_theta, R_TWO*cos_theta,  R_TWO*cos_theta,
    0, 0,  R_TWO*omega_sum*cos_theta, R_TWO*sin_theta,  R_TWO*sin_theta,
    0, 0,  0,                         -R_L,              R_L,
    0, 0,  0,                         -m_params.a,       0,
    0, 0,  0,                          0,                -m_params.a;
}

//discretize
template<std::size_t V, std::size_t F>
void MPCController<V, F>::discretize() {
    // ZOH discretization via matrix exponential
    Eigen::Matrix<double, n_states + m_inputs, n_states + m_inputs> m_exp;
    m_exp.setZero();
    m_exp.template block<n_states, n_states>(0, 0) = m_Ac;
    m_exp.template block<n_states, m_inputs>(0, n_states) = m_Bc;
    m_exp *= m_params.h;
    Eigen::Matrix<double, n_states + m_inputs, n_states + m_inputs> Md = m_exp.exp();
    m_A = Md.template block<n_states, n_states>(0, 0);
    m_B = Md.template block<n_states, m_inputs>(0, n_states);
}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildStageModels() {
    Eigen::Matrix<double, n_states + m_inputs, n_states + m_inputs> Mexp;

    for (std::size_t k = 0; k < F; k++) {
        double theta = m_x_ref(2, k);
        double wl = m_x_ref(3, k), wr = m_x_ref(4, k);
        double omega_sum = wl + wr;
        constexpr double OMEGA_SUM_EPS = 2.0;
        if (std::abs(omega_sum) < OMEGA_SUM_EPS)
            omega_sum = (omega_sum >= 0.0) ? OMEGA_SUM_EPS : -OMEGA_SUM_EPS;

        double c = std::cos(theta), s = std::sin(theta);
        m_Ac 
            0, 0, -R_TWO*omega_sum*s,  R_TWO*c,     R_TWO*c,
            0, 0,  R_TWO*omega_sum*c,  R_TWO*s,     R_TWO*s,
            0, 0,  0,                 -R_L,         R_L,
            0, 0,  0,                 -m_params.a,  0,
            0, 0,  0,                  0,          -m_params.a;

        Mexp.setZero();
        Mexp.template block<n_states, n_states>(0, 0) = m_Ac;
        Mexp.template block<n_states, m_inputs>(0, n_states) = m_Bc;
        Mexp *= m_params.h;
        auto Md = Mexp.exp().eval();
        m_A_k[k] = Md.template block<n_states, n_states>(0, 0);
        m_B_k[k] = Md.template block<n_states, m_inputs>(0, n_states);

        m_d_k[k] = m_x_ref.col(k + 1)
                 - m_A_k[k] * m_x_ref.col(k)
                 - m_B_k[k] * m_u_ref.col(k);
    }
}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildPredictionMatrices() {
    Eigen::Matrix<double, n_states, n_states> Phi = m_A_k[0]; 
    Eigen::Matrix<double, n_states, 1> w = m_d_k[0];  
    std::array<Eigen::Matrix<double, n_states, m_inputs>, V> E;

    for (std::size_t i = 0; i < F; i++) {
        if (i > 0) {
            Phi = m_A_k[i] * Phi;
            w = m_A_k[i] * w + m_d_k[i];
            for (std::size_t j = 0; j < std::min(i, V); j++) {
                E[j] = m_A_k[i] * E[j];
            }
        }
        if (i < V) {
            E[i] = m_B_k[i];
        }
        else {
            E[V - 1] += m_B_k[i];   
        }


        m_O.template block<r_states, n_states>(i * r_states, 0) = m_C * Phi;
        m_O_xy.template block<2, n_states>(i * 2, 0) = m_C_xy * Phi;
        m_O_omega.template block<2, n_states>(i * 2, 0) = m_C_omega * Phi;


        m_D_z.template segment<r_states>(i * r_states) = m_C * w;
        m_D_xy.template segment<2>(i * 2) = m_C_xy * w;
        m_D_omega.template segment<2>(i * 2) = m_C_omega * w;


        for (std::size_t j = 0; j <= std::min(i, V - 1); j++) {
            m_M.template block<r_states, m_inputs>(i * r_states, j * m_inputs) = m_C * E[j];
            m_M_xy.template block<2, m_inputs>(i * 2, j * m_inputs) = m_C_xy * E[j];
            m_M_omega.template block<2, m_inputs>(i * 2, j * m_inputs) = m_C_omega * E[j];
        }
    }
}

// //build O and M prediction matrices
// template<std::size_t V, std::size_t F>
// void MPCController<V, F>::buildPredictionMatrices() {
//     //makes matrix to hold A to a power
//     Eigen::Matrix<double, n_states, n_states> m_A_power = m_A;
//     //makes O matrices
//     for (std::size_t i = 0; i<F; i++){
//         //output prediction: y_i = C * A^i * x_0
//         m_O.template block<r_states, n_states> (i*r_states, 0) = m_C * m_A_power;
//         //xy position component
//         m_O_xy.template block<2, n_states>(i*2, 0) = m_C_xy * m_A_power;
//         //omega component
//         m_O_omega.template block<2, n_states>(i*2, 0) = m_C_omega * m_A_power;
//         //incr power for next step
//         m_A_power *= m_A ;
//     }
//     //makes M matrices
//     for (std::size_t i = 0; i < F; i++) {
//         for (std::size_t j = 0; j < V; j++) {
//             // upper triangular zero region
//             if (j > i) {
//                 continue;
//             }
//             // Compute A^(i-j)
//             Eigen::Matrix<double, n_states, n_states> A_term = 
//                 Eigen::Matrix<double, n_states, n_states>::Identity();
            
//             //normal region: each control move is free
//             //applies when prediction step i is within control horizon,
//             //or when we're before the last free control move
//             if (i < V || j < V - 1) {
//                 for (std::size_t k = 0; k < (i - j); k++) {
//                     A_term *= m_A;
//                 }
//                 m_M.template block<r_states, m_inputs>(i * r_states, j * m_inputs) = 
//                     m_C * A_term * m_B;
//                 m_M_xy.template block<2, m_inputs>(i * 2, j * m_inputs) = 
//                     m_C_xy * A_term * m_B;
//                 m_M_omega.template block<2, m_inputs>(i * 2, j * m_inputs) = 
//                     m_C_omega * A_term * m_B;
//             }
//             //held control region: last control move u_{V-1} is held constant
//             //for all remaining prediction steps i >= V
//             else {
//                 //A_bar = I + A + A² + ... + A^(i-V+1)
//                 //sums the effect of holding u_{V-1} over multiple steps
//                 Eigen::Matrix<double, n_states, n_states> A_bar = 
//                     Eigen::Matrix<double, n_states, n_states>::Zero();
//                 Eigen::Matrix<double, n_states, n_states> A_sum = 
//                     Eigen::Matrix<double, n_states, n_states>::Identity();
//                 for (std::size_t k = 0; k <= (i - V + 1); k++) {
//                     if (k > 0) {
//                         A_sum *= m_A;
//                     }
//                     A_bar += A_sum;
//                 }
//                 for (std::size_t k = 0; k < (V - 1 - j); k++) {
//                     A_term *= m_A;
//                 }

//                 //A_term already has A^(i-j) where j = V-1, so i-j = i-(V-1)
//                 //we need A^(i-(V-1)) * A_bar * B for the held effect
//                 m_M.template block<r_states, m_inputs>(i * r_states, j * m_inputs) = 
//                     m_C * A_term * A_bar * m_B;
//                 m_M_xy.template block<2, m_inputs>(i * 2, j * m_inputs) = 
//                     m_C_xy * A_term * A_bar * m_B;
//                 m_M_omega.template block<2, m_inputs>(i * 2, j * m_inputs) = 
//                     m_C_omega * A_term * A_bar * m_B;
//             }
//         }
//     }
// }

//assemble full G, b: stacks all inequality constraints into single matrix G and vector b
template<std::size_t V, std::size_t F>
void MPCController<V, F>::assembleConstraints(double V_batt, double I_total, const Eigen::Matrix<double, m_inputs, 1> u_prev) {
    //rebuild time-varying constr
    buildConstraintDelta(u_prev);
    buildConstraintBattery(V_batt, I_total);
    buildConstraintPosition();
    buildConstraintOmega();
    //build G and b:
    int row = 0;

    // Delta_u constraints
    m_G.middleRows(row, 2*m_inputs*V) = m_G_delta;
    m_b.segment(row, 2*m_inputs*V) = m_b_delta;
    row += 2*m_inputs*V;

    // input constraints
    m_G.middleRows(row, 2*m_inputs*V) = m_G_u;
    m_b.segment(row, 2*m_inputs*V) = m_b_u;
    row += 2*m_inputs*V;

    // battery constraints
    m_G.middleRows(row, 2*m_inputs*V) = m_G_b;
    m_b.segment(row, 2*m_inputs*V) = m_b_b;
    row += 2*m_inputs*V;

    // position constraints
    m_G.middleRows(row, 4*F) = m_G_z_xy;
    m_b.segment(row, 4*F) = m_b_z_xy;
    row += 4*F;

    // omega constraints
    m_G.middleRows(row, 4*F) = m_G_z_omega;
    m_b.segment(row, 4*F) = m_b_z_omega;
}

//delta u constraint: limits how fast voltage can change: |u_k - u_{k-1}| ≤ delta_u_max
template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildConstraintDelta(
    const Eigen::Matrix<double, m_inputs, 1> u_prev) {
    // base limits from params
    m_b_delta.template topRows<m_inputs*V>().setConstant(-m_params.delta_u_min);
    m_b_delta.template bottomRows<m_inputs*V>().setConstant(m_params.delta_u_max);
    //adjust first block for actual previous control
    m_b_delta.template segment<m_inputs>(0) += u_prev; //lower bound
    m_b_delta.template segment<m_inputs>(m_inputs * V) += u_prev; //upper bound
}

//battery voltage constraint: available voltage drops with current: V_avail = V_batt - I·R_int
template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildConstraintBattery(double V_battery, double I_total) {
    double u_b_max = V_battery - m_params.R_internal * I_total;
    //symmetric limit for both motors, at all steps 
    m_b_b.template topRows<m_inputs*V>().setConstant(u_b_max);
    m_b_b.template bottomRows<m_inputs*V>().setConstant(u_b_max);
}

// field position constraint as soft field boundaries: computed as output constraint on predicted positions
template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildConstraintPosition() {
    m_b_z_xy.topRows(2 * F) = m_z_xy_max + (m_O_xy * m_x_hat + m_D_xy);
    m_b_z_xy.bottomRows(2 * F) = m_z_xy_max - (m_O_xy * m_x_hat + m_D_xy);
    m_G_z_xy.topRows(2*F) = -m_M_xy;
    m_G_z_xy.bottomRows(2*F) =  m_M_xy;
}

// motor speed constraint
template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildConstraintOmega() {
    //initialize b_z,omega
    m_b_z_omega.topRows(2 * F) = m_z_omega_max + (m_O_omega * m_x_hat + m_D_omega);
    m_b_z_omega.bottomRows(2 * F) = m_z_omega_max - (m_O_omega * m_x_hat + m_D_omega);
    m_G_z_omega.topRows(2*F) = -m_M_omega;
    m_G_z_omega.bottomRows(2*F) =  m_M_omega;
}

//Eigen sparse to OSQP CSC (OSQP needs compressed sparse column format)
template<std::size_t V, std::size_t F>
typename MPCController<V, F>::CscStorage MPCController<V, F>::eigenToCSC(Eigen::SparseMatrix<double>& m) {
    //ensure single allocation w/ no gaps
    m.makeCompressed();
    
    //output struct holds OSQP compatible vectors
    CscStorage out;

    //get matrix dimensions
    const int nnz  = m.nonZeros();
    const int rows = m.rows();
    const int cols = m.cols();

    //allocate OSQP arrays 
    out.values.resize(nnz);
    out.row_idx.resize(nnz);
    out.col_ptr.resize(cols + 1);

    //copy non zero values and their row indices
    for (int k = 0; k < nnz; k++) {
        //m.valuePtr(): raw array of non zero doubles
        out.values[k]  = static_cast<OSQPFloat>(m.valuePtr()[k]);
        //m.innerIndexPtr(): row indices corresponding to values
        out.row_idx[k] = static_cast<OSQPInt>(m.innerIndexPtr()[k]);
    }
    //copy column pointers
    for (int k = 0; k <= cols; k++) {
        // m.outerIndexPtr()[k] = index in values/row_idx where column k starts
        out.col_ptr[k] = static_cast<OSQPInt>(m.outerIndexPtr()[k]);
    }

    //OSQP struct points to our vectors
    out.mat.m     = rows;
    out.mat.n     = cols;
    out.mat.nz    = -1; //CSC format flag (not triplet)
    out.mat.nzmax = nnz; //allocated capacity
    out.mat.x     = out.values.data(); //value array pointer
    out.mat.i     = out.row_idx.data(); //row index pointer
    out.mat.p     = out.col_ptr.data(); //column pointer array

    return out;
}

//build QP matrices: forms condensed QP:  min_U  1/2 U^T*P*U + q^T*U: P = M^T*W_4*M + W_3; q = -M^T*W_4*s: s = z_des - O*x_hat
template<std::size_t V, std::size_t F>
void MPCController<V, F>::assembleQP() {
    // tracking error: s = z_desired - free response
    m_s.noalias() = m_z_desired - m_O * m_x_hat - m_D_z;
    for (std::size_t i = 0; i < F; i++) {
        double& theta_err = m_s(i * r_states + 2);
        while (theta_err > M_PI) theta_err -= 2.0 * M_PI;
        while (theta_err < -M_PI) theta_err += 2.0 * M_PI;
    }
    //P = M^T*W_4*M (Hessian)
    Eigen::Matrix<double, r_states*F, m_inputs*V> m_W_four_M;
    m_W_four_M.noalias() = m_W_four * m_M;
    m_P.noalias() = m_M.transpose() * m_W_four_M + m_W_three;
    //regularization for stability
    m_P.diagonal().array() += 2;

    //q = -M^T*W_4*s (linear term)
    Eigen::Matrix<double, r_states*F, 1> m_W_four_s;
    m_W_four_s.noalias() = m_W_four * m_s;
    m_q.noalias() = -m_M.transpose() * m_W_four_s;  
}

template<std::size_t V, std::size_t F>
template<typename Derived>
void MPCController<V, F>::fillCscDense(CscStorage& out,
        const Eigen::MatrixBase<Derived>& M, bool upperOnly) {
    std::size_t k = 0;
    for (int c = 0; c < M.cols(); c++) {
        int rmax = upperOnly ? std::min<int>(c, M.rows() - 1) : M.rows() - 1;
        for (int r = 0; r <= rmax; r++)
            out.values[k++] = static_cast<OSQPFloat>(M(r, c));
    }
}



template<std::size_t V, std::size_t F>
void MPCController<V, F>::initCscDense(CscStorage& out, int rows, int cols, bool upperOnly) {
    out.values.clear(); out.row_idx.clear(); out.col_ptr.clear();
    out.col_ptr.push_back(0);
    for (int c = 0; c < cols; c++) {
        int rmax = upperOnly ? std::min(c, rows - 1) : rows - 1;
        for (int r = 0; r <= rmax; r++) {
            out.row_idx.push_back(static_cast<OSQPInt>(r));
            out.values.push_back(0.0);
        }
        out.col_ptr.push_back(static_cast<OSQPInt>(out.row_idx.size()));
    }
    out.mat.m = rows;  out.mat.n = cols;  out.mat.nz = -1;
    out.mat.nzmax = static_cast<OSQPInt>(out.values.size());
    out.mat.x = out.values.data();
    out.mat.i = out.row_idx.data();
    out.mat.p = out.col_ptr.data();
}

//calls OSQP to solve constrained QP, extracts first optimal input
template<std::size_t V, std::size_t F>
void MPCController<V, F>::solveQP() {
    constexpr int N_VARS = m_inputs * V;
    constexpr int N_CONSTR = 6 * m_inputs * V + 8 * F;

    if (!m_solver) {
        initCscDense(m_P_csc, N_VARS, N_VARS, true);
        initCscDense(m_A_full_csc, N_CONSTR, N_VARS, false);
        fillCscDense(m_P_csc, m_P, true);
        fillCscDense(m_A_full_csc, m_G, false);

        m_q_v.resize(N_VARS);
        m_l_v.assign(N_CONSTR, -OSQP_INFTY);
        m_u_v.resize(N_CONSTR);
        for (int i = 0; i < N_VARS; i++) m_q_v[i] = static_cast<OSQPFloat>(m_q(i));
        for (int i = 0; i < N_CONSTR; i++) m_u_v[i] = static_cast<OSQPFloat>(m_b(i));

        OSQPSettings settings;
        osqp_set_default_settings(&settings);
        settings.warm_starting = 1;   
        settings.max_iter = 4000;
        settings.polishing = 0;   
        settings.eps_abs = 1e-3;
        settings.eps_rel = 1e-3;
        settings.verbose = 0;

        if (osqp_setup(&m_solver, &m_P_csc.mat, m_q_v.data(), &m_A_full_csc.mat, m_l_v.data(), m_u_v.data(), N_CONSTR, N_VARS, &settings) != 0) {
            m_solver = nullptr;
            throw std::runtime_error("OSQP setup failed");
        }
    } 
    else {
        fillCscDense(m_P_csc, m_P, true);
        fillCscDense(m_A_full_csc, m_G, false);
        for (int i = 0; i < N_VARS; i++) m_q_v[i] = static_cast<OSQPFloat>(m_q(i));
        for (int i = 0; i < N_CONSTR; i++) m_u_v[i] = static_cast<OSQPFloat>(m_b(i));

        osqp_update_data_mat(m_solver, m_P_csc.values.data(), OSQP_NULL, m_P_csc.mat.nzmax, m_A_full_csc.values.data(), OSQP_NULL, m_A_full_csc.mat.nzmax);
        osqp_update_data_vec(m_solver, m_q_v.data(), OSQP_NULL, m_u_v.data());
    }

    OSQPInt flag = osqp_solve(m_solver);
    OSQPInt status = m_solver->info->status_val;
    if (flag != 0 || (status != OSQP_SOLVED && status != OSQP_SOLVED_INACCURATE)) {
        std::cerr << "OSQP status " << status << " iter=" << m_solver->info->iter << "\n";
        throw std::runtime_error("OSQP solve failed");   
    }

    m_u_left = m_solver->solution->x[0];
    m_u_right = m_solver->solution->x[1];
    m_u_prev << m_u_left, m_u_right;

    static std::vector<OSQPFloat> x_ws(N_VARS);
    for (int i = 0; i + m_inputs < N_VARS; i++) {
        x_ws[i] = m_solver->solution->x[i + m_inputs];
    }
    for (int i = N_VARS - m_inputs; i < N_VARS; i++) {
        x_ws[i] = m_solver->solution->x[i];
    }
    osqp_warm_start(m_solver, x_ws.data(), OSQP_NULL);
}

//outputs wheel voltages using MPC, taking in info from MPCUpdatePacket
template<std::size_t V, std::size_t F>
WheelVelocities MPCController<V, F>::compute(const Pose& currentPose,
        const Eigen::Matrix<double, r_states*(F+1), 1>& z_ref,
        double omega_L, double omega_R, double V_battery, double I_total)
{
    m_z_ref = z_ref;
    m_z_desired = z_ref.template segment<r_states * F>(r_states); 
    m_x_hat << currentPose.x, currentPose.y, currentPose.theta, omega_L, omega_R;

    buildReferenceStates();
    buildStageModels();
    buildPredictionMatrices();
    assembleConstraints(V_battery, I_total, m_u_prev);
    assembleQP();
    solveQP();
    return WheelVelocities{m_u_left, m_u_right, ControlMode::INPUT_VOLTAGE};
}

//! Unused override that doesn't work (no planned use either)
template<std::size_t V, std::size_t F>
WheelVelocities MPCController<V, F>::compute(const Pose& currentPose, const ALS_Path& als_path, std::size_t& closestSampleIdx) {
    std::cerr << "USING UNSUPPPORTED OVERLOAD OF COMPUTE!!!" <<  std::endl;
    return {0,0};
}

//clears all dynamic data for fresh start (ie. new path segment)
template<std::size_t V, std::size_t F>
void MPCController<V, F>::reset() {
    m_x_hat.setZero();
    m_z_desired.setZero();
    m_s.setZero();
    m_q.setZero();
    m_P.setZero();
    m_b.setZero();
    m_b_delta.setZero();
    m_b_b.setZero();
    m_b_z_xy.setZero();
    m_b_z_omega.setZero();
    m_u_left = 0.0;
    m_u_right = 0.0;
}


//unpack z_desired from float array
template<std::size_t V, std::size_t F>
auto MPCController<V, F>::unpackZDesired(const float* z_raw) {
    Eigen::Matrix<double, r_states * (F + 1), 1> z;
    for (std::size_t i = 0; i < F + 1; i++) {
        const std::size_t base = i * 3;
        z(base + 0) = static_cast<double>(z_raw[base + 0]);
        z(base + 1) = static_cast<double>(z_raw[base + 1]);
        z(base + 2) = static_cast<double>(z_raw[base + 2]);
    }
    return z;
}

//function to recieve update packet: compute voltages: send control packet
template<std::size_t V, std::size_t F>
void MPCController<V, F>::MPCControl(SerialProtocol& serial, MPCController& mpc) {
    //wait for state packet from vex brain
std::optional<MPCUpdatePacket> packet_opt = serial.receive<MPCUpdatePacket>(SerialProtocol::PacketType::MPC_UPDATE);
    //timeout or bad packet
    if(!packet_opt.has_value()) {
        return;
    }

    const MPCUpdatePacket& packet = *packet_opt;

    //unpack pose
    Pose pose{
        packet.pose_x,
        packet.pose_y,
        packet.pose_theta
    };

    //unpack reference trajectory
    auto z_desired = unpackZDesired(packet.z_desired);

    //initialize MPCControlPacket with default failsafe of 0V
    MPCControlPacket response{0.0f, 0.0f};
    //run MPC optimization with exception safety
    try {
        WheelVelocities out = mpc.compute(pose, z_desired, packet.omega_L, packet.omega_R, packet.V_battery, packet.I_total);
    //pack optimal voltages on success
    response.V_left = static_cast<float>(out.left);
    response.V_right = static_cast<float>(out.right);
    }
    catch (const std::exception& e){
        //MPC failed: log error and send zero volts
        std::cerr << "MPC ERROR: " << e.what() << ", sending zero volts\n";
    }
    //send optimal voltages
    serial.send(SerialProtocol::PacketType::MPC_CONTROL, response);
    std::cerr << "sent V_L=" << response.V_left << " V_R=" << response.V_right<<  std::endl;
}

template<std::size_t V, std::size_t F>
MPCController<V, F>::~MPCController() {
    if (m_solver) {
        osqp_cleanup(m_solver);
    }
}

//explicit instantiation for V=F=15
template class MPCController<15, 15>;

