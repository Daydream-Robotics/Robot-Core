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
    : m_params(params) {
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

//w_rap angle to (-pi, pi]
inline double wrapAngle(double angle) {
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

//build per-stage LTV models: linearize + discretize about each ref state
template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildStageModels() {
    Eigen::Matrix<double, n_states + m_inputs, n_states + m_inputs> m_exp;

    for (std::size_t i = 0; i < F; i++) {
        //ref heading and wheel speed for the stage
        double theta = m_x_ref(2, i);
        double w_l = m_x_ref(3, i);
        double w_r = m_x_ref(4, i);
        //omega sum so theta col remains nonzero
        double omega_sum = w_l + w_r;
        constexpr double OMEGA_SUM_EPS = 2.0;
        if (std::abs(omega_sum) < OMEGA_SUM_EPS) {
            omega_sum = (omega_sum >= 0.0) ? OMEGA_SUM_EPS : -OMEGA_SUM_EPS;
        }
        //cont Jacobian at stage-i ref
        double c = std::cos(theta);
	    double s = std::sin(theta);
        m_Ac <<
            0, 0, -R_TWO*omega_sum*s,  R_TWO*c,     R_TWO*c,
            0, 0,  R_TWO*omega_sum*c,  R_TWO*s,     R_TWO*s,
            0, 0,  0,                 -R_L,         R_L,
            0, 0,  0,                 -m_params.a,  0,
            0, 0,  0,                  0,          -m_params.a;
        //ZOH discretize the curr stage model
        m_exp.setZero();
        m_exp.template block<n_states, n_states>(0, 0) = m_Ac;
        m_exp.template block<n_states, m_inputs>(0, n_states) = m_Bc;
        m_exp *= m_params.h;
        auto m_d = m_exp.exp().eval();
        m_A_k[k] = m_d.template block<n_states, n_states>(0, 0);
        m_B_k[k] = m_d.template block<n_states, m_inputs>(0, n_states);

        //affine term  d_k = x_ref[k+1] - A_k*x_ref[k] - B_k*u_ref[k]
        m_d_k[k] = m_x_ref.col(k + 1)
                 - m_A_k[k] * m_x_ref.col(k)
                 - m_B_k[k] * m_u_ref.col(k);
    }
}

//reconstruct full reference states (x,y,theta,omega_L,omega_R) and
//feedforward voltages from the F+1 reference poses
template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildReferenceStates() {
    const double h = m_params.h;
    const double r = m_params.r;
    const double L = m_params.L;

    for (std::size_t i = 0; i < F; i++) {
        //consecutive ref poses i and i+1
        double x_zero  = m_z_desired(3*i + 0);
        double y_zero  = m_z_desired(3*i + 1);
        double theta_zero  = m_z_desired(3*i + 2);
        double x_one  = m_z_desired(3*(i+1) + 0);
        double y_one  = m_z_desired(3*(i+1) + 1);
        double theta_one = m_z_desired(3*(i+1) + 2);

        //signed forward speed: displacement projected onto heading
        double v = ((x_one - x_zero) * std::cos(theta_zero) + (y_one - y_zero) * std::sin(th)) / h;
        //heading rate from finite difference
        double w = wrapAngle(theta_one - theta_zero) / h;

        //diff-drive inverse kinematics: wheel omegas from (v, w)
        double w_l = (v - w * L / 2.0) / r;
        double w_r = (v + w * L / 2.0) / r;

        m_x_ref.col(k) << x_zero, y_zero, theta_zero, w_l, w_r;
        //steady-state feedforward voltage
        m_u_ref.col(k) << (m_params.a / m_params.b) * w_l,
                          (m_params.a / m_params.b) * w_r;
    }
    //terminal reference state: last pose, hold last wheel speeds
    m_x_ref.col(F) << m_z_desired(3*F + 0), m_z_desired(3*F + 1), m_z_desired(3*F + 2),
                      m_x_ref(3, F - 1), m_x_ref(4, F - 1);
}

//build LTV pred. matrices over the horizon
template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildPredictionMatrices() {
    //phi is the state trasition and w is the accumulated affine terms
    Eigen::Matrix<double, n_states, n_states> m_Phi = m_A_k[0]; 
    Eigen::Matrix<double, n_states, 1> m_w = m_d_k[0];  
    //m_E is effect of input on the state at the current step
    std::array<Eigen::Matrix<double, n_states, m_inputs>, V> m_E;

    for (std::size_t i = 0; i < F; i++) {
        if (i > 0) {
            //propagate transition, affine term, and all input effects
            m_Phi = m_A_k[i] * m_Phi;
            m_w = m_A_k[i] * m_w + m_d_k[i];
            for (std::size_t j = 0; j < std::min(i, V); j++) {
                m_E[j] = m_A_k[i] * m_E[j];
            }
        }
        if (i < V) {
            //new free control move enters at step i
            m_E[i] = m_B_k[i];
        }
        else {
            //past the control horizon: held move u_{V-1} keeps injecting B_k
            m_E[V - 1] += m_B_k[i];   
        }

        //free-response rows: O = C * Phi (full, xy-only, omega-only outputs)
        m_O.template block<r_states, n_states>(i * r_states, 0) = m_C * m_Phi;
        m_O_xy.template block<2, n_states>(i * 2, 0) = m_C_xy * m_Phi;
        m_O_omega.template block<2, n_states>(i * 2, 0) = m_C_omega * m_Phi;

        //affine-response rows: D = C * w
        m_D_z.template segment<r_states>(i * r_states) = m_C * m_w;
        m_D_xy.template segment<2>(i * 2) = m_C_xy * m_w;
        m_D_omega.template segment<2>(i * 2) = m_C_omega * m_w;

        //forced-response rows: M[i][j] = C * E[j] for every active input
        for (std::size_t j = 0; j <= std::min(i, V - 1); j++) {
            m_M.template block<r_states, m_inputs>(i * r_states, j * m_inputs) = m_C * m_E[j];
            m_M_xy.template block<2, m_inputs>(i * 2, j * m_inputs) = m_C_xy * m_E[j];
            m_M_omega.template block<2, m_inputs>(i * 2, j * m_inputs) = m_C_omega * m_E[j];
        }
    }
}

//assemble full G, b: stacks all inequality constraints into single matrix G and vector b
template<std::size_t V, std::size_t F>
void MPCController<V, F>::assembleConstraints(double V_batt, double I_total, const Eigen::Matrix<double, m_inputs, 1> u_prev) {
    //rebuild time-varying constr
    buildConstraintDelta(u_prev);
    buildConstraintBattery(V_batt, I_total);
    buildConstraintPosition();
    buildConstraintOmega();
    //build G and b: stacking the blocks
    int row = 0;

    // Delta_u constraints
    m_G.middleRows(row, 2 * m_inputs * V) = m_G_delta;
    m_b.segment(row, 2 * m_inputs * V) = m_b_delta;
    row += 2 * m_inputs * V;

    // input constraints
    m_G.middleRows(row, 2 * m_inputs * V) = m_G_u;
    m_b.segment(row, 2 * m_inputs * V) = m_b_u;
    row += 2 * m_inputs * V;

    // battery constraints
    m_G.middleRows(row, 2 * m_inputs * V) = m_G_b;
    m_b.segment(row, 2 * m_inputs * V) = m_b_b;
    row += 2 * m_inputs * V;

    // position constraints
    m_G.middleRows(row, 4 * F) = m_G_z_xy;
    m_b.segment(row, 4 * F) = m_b_z_xy;
    row += 4 * F;

    // omega constraints
    m_G.middleRows(row, 4 * F) = m_G_z_omega;
    m_b.segment(row, 4 * F) = m_b_z_omega;
}

//delta u constraint: limits how fast voltage can change: |u_k - u_{k-1}| <= delta_u_max
template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildConstraintDelta(const Eigen::Matrix<double, m_inputs, 1> u_prev) {
    // base limits from params
    m_b_delta.template topRows<m_inputs*V>().setConstant(-m_params.delta_u_min);
    m_b_delta.template bottomRows<m_inputs*V>().setConstant(m_params.delta_u_max);
    //adjust first block for actual previous control
    m_b_delta.template segment<m_inputs>(0) += u_prev; //lower bound
    m_b_delta.template segment<m_inputs>(m_inputs * V) += u_prev; //upper bound
}

//battery voltage constraint: available voltage drops with current: V_avail = V_batt - I*R_int
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
    m_G_z_xy.topRows(2 * F) = -m_M_xy;
    m_G_z_xy.bottomRows(2 * F) =  m_M_xy;
}

// motor speed constraint
template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildConstraintOmega() {
    //initialize b_z,omega
    m_b_z_omega.topRows(2 * F) = m_z_omega_max + (m_O_omega * m_x_hat + m_D_omega);
    m_b_z_omega.bottomRows(2 * F) = m_z_omega_max - (m_O_omega * m_x_hat + m_D_omega);
    m_G_z_omega.topRows(2 * F) = -m_M_omega;
    m_G_z_omega.bottomRows(2 * F) =  m_M_omega;
}


//build QP matrices: forms condensed QP:  min_U  1/2 U^T*P*U + q^T*U: P = M^T*W_4*M + W_3; q = -M^T*W_4*s: s = z_des - O*x_hat
template<std::size_t V, std::size_t F>
void MPCController<V, F>::assembleQP() {
    // tracking error: s = z_desired - free response
    m_s.noalias() = m_z_desired.template segment<r_states * F>(r_states) - m_O * m_x_hat - m_D_z;
    //wrap every heading error to (-pi, pi]
    for (std::size_t i = 0; i < F; i++) {
        m_s(i * r_states + 2) = wrapAngle(m_s(i * r_states + 2));
    }
    //P = M^T*W_4*M (Hessian)
    Eigen::Matrix<double, r_states*F, m_inputs*V> m_W_four_M;
    m_W_four_M.noalias() = m_W_four * m_M;
    m_P.noalias() = m_M.transpose() * m_W_four_M + m_W_three;
    //regularization for stability
    m_P.diagonal().array() += 1e-2;

    //q = -M^T*W_4*s (linear term)
    Eigen::Matrix<double, r_states*F, 1> m_W_four_s;
    m_W_four_s.noalias() = m_W_four * m_s;
    m_q.noalias() = -m_M.transpose() * m_W_four_s;  
    //cross term so the first-move penalty measures u_0 - u_prev
    m_q.template segment<m_inputs>(0) -=
    m_params.q_zero_multiplier * m_params.q_u * m_u_prev;
}

//copy a dense Eigen matrix into a preallocated CSC 
template<std::size_t V, std::size_t F>
template<typename Derived>
void MPCController<V, F>::fillCscDense(CscStorage& out, const Eigen::MatrixBase<Derived>& m_M, bool upperOnly) {
    std::size_t k = 0;
    //column-major walk; upperOnly stores just the upper triangle (OSQP P matrix)
    for (int c = 0; c < m_M.cols(); c++) {
        int r_max = upperOnly ? std::min<int>(c, m_M.rows() - 1) : m_M.rows() - 1;
        for (int r = 0; r <= r_max; r++) {
            out.values[k++] = static_cast<OSQPFloat>(m_M(r, c));
        }
    }
}

 
//allocate a dense-pattern CSC, setting up the row/col index arrays once so later updates only rewrite values
template<std::size_t V, std::size_t F>
void MPCController<V, F>::initCscDense(CscStorage& out, int rows, int cols, bool upperOnly) {
    out.values.clear(); out.row_idx.clear(); out.col_ptr.clear();
    out.col_ptr.push_back(0);
    //build column-by-column where every entry is a stored zero
    for (int c = 0; c < cols; c++) {
        int r_max = upperOnly ? std::min(c, rows - 1) : rows - 1;
        for (int r = 0; r <= r_max; r++) {
            out.row_idx.push_back(static_cast<OSQPInt>(r));
            out.values.push_back(0.0);
        }
        out.col_ptr.push_back(static_cast<OSQPInt>(out.row_idx.size()));
    }
    //fill the OSQP matrix header with pointers into vectors
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

    //first call build CSC storage, bounds, settings, and OSQP workspace
    if (!m_solver) {
        initCscDense(m_P_csc, N_VARS, N_VARS, true);
        initCscDense(m_A_full_csc, N_CONSTR, N_VARS, false);
        fillCscDense(m_P_csc, m_P, true);
        fillCscDense(m_A_full_csc, m_G, false);

        //bounds: G*U <= b encoded as -inf <= G*U <= b
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
    //subsequent calls reuse workspace and just push new matrix values and vectors
    else {
        fillCscDense(m_P_csc, m_P, true);
        fillCscDense(m_A_full_csc, m_G, false);
        for (int i = 0; i < N_VARS; i++) m_q_v[i] = static_cast<OSQPFloat>(m_q(i));
        for (int i = 0; i < N_CONSTR; i++) m_u_v[i] = static_cast<OSQPFloat>(m_b(i));

        osqp_update_data_mat(m_solver, m_P_csc.values.data(), OSQP_NULL, m_P_csc.mat.nzmax, m_A_full_csc.values.data(), OSQP_NULL, m_A_full_csc.mat.nzmax);
        osqp_update_data_vec(m_solver, m_q_v.data(), OSQP_NULL, m_u_v.data());
    }

    //solve
    OSQPInt flag = osqp_solve(m_solver);
    OSQPInt status = m_solver->info->status_val;
    if (flag != 0 || (status != OSQP_SOLVED && status != OSQP_SOLVED_INACCURATE)) {
        std::cerr << "OSQP status " << status << " iter=" << m_solver->info->iter << "\n";
        throw std::runtime_error("OSQP solve failed");   
    }

    //first stage optimal input (u_0)
    double u0_left = m_solver->solution->x[0];
    double u0_right = m_solver->solution->x[1];

    // OSQP_SOLVED_INACCURATE can return a stage-0 input that violates the hard delta-u rate limit so we treat that the same as a failed solve
    // numerical tolerance above the hard limit
    constexpr double kDeltaSlack = 2;
    double d_left = u0_left - m_u_prev(0);
    double d_right = u0_right - m_u_prev(1);
    if (d_left  > m_params.delta_u_max + kDeltaSlack || d_left  < m_params.delta_u_min - kDeltaSlack || d_right > m_params.delta_u_max + kDeltaSlack || d_right < m_params.delta_u_min - kDeltaSlack) {
        std::cerr << "OSQP solution violates delta-u limit (d_left=" << d_left << " d_right=" << d_right << ")\n";
        throw std::runtime_error("OSQP solution violates rate limit");
    }

    //store applied input for next cycle's delta-u constraint and cost
    m_u_left = u0_left;
    m_u_right = u0_right;
    m_u_prev << m_u_left, m_u_right;

    //warm start next solve: shift solution one stage left and repeat last stage
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
WheelVelocities MPCController<V, F>::compute(const Pose& currentPose, const Eigen::Matrix<double, r_states*(F+1), 1>& z_ref, double omega_L, double omega_R, double V_battery, double I_total) {
     //store reference trajectory and current state estimate
    m_z_ref = z_ref;
    m_z_desired = z_ref;
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
    m_u_prev.setZero();
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


