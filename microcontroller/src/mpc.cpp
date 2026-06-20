#include <iostream>
#include "mpc.hpp"
#define EIGEN_STACK_ALLOCATION_LIMIT 1048576 
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
    m_Ac << 
    0, 0, -R_TWO*(omega_L+omega_R)*sin_theta, R_TWO*cos_theta, R_TWO*cos_theta,
    0,0,   R_TWO*(omega_L+omega_R)*cos_theta, R_TWO*sin_theta, R_TWO*sin_theta,
    0,0,0, -R_L, R_L,
    0,0,0, -m_params.a, 0,
    0,0,0,0, -m_params.a;
}

//discretize via matrix exponential
template<std::size_t V, std::size_t F>
void MPCController<V, F>::discretize() {
    //declares a matrix to hold exp version of continuous A&B matrices
    Eigen::Matrix<double, n_states + m_inputs, n_states + m_inputs> m_exp;
    m_exp.setZero();
    //puts A^C_K and B^c in exp matrix
    m_exp.template block<n_states, n_states>(0, 0) = m_Ac;
    m_exp.template block<n_states, m_inputs>(0, n_states) = m_Bc;
    //multiplies exp matrix by sample period
    m_exp *= m_params.h;
    // exponentiates matrix
    auto Md = m_exp.exp();
    // extract A_k
    m_A = Md.template block<n_states, n_states>(0, 0);
    // extract B_k
    m_B = Md.template block<n_states, m_inputs>(0, n_states);
}

//build O and M prediction matrices
template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildPredictionMatrices() {
    //makes matrix to hold A to a power
    Eigen::Matrix<double, n_states, n_states> m_A_power = m_A;
    //makes O matrices
    for (std::size_t i = 0; i<F; i++){
        //output prediction: y_i = C * A^i * x_0
        m_O.template block<r_states, n_states> (i*r_states, 0) = m_C * m_A_power;
        //xy position component
        m_O_xy.template block<2, n_states>(i*2, 0) = m_C_xy * m_A_power;
        //omega component
        m_O_omega.template block<2, n_states>(i*2, 0) = m_C_omega * m_A_power;
        //incr power for next step
        m_A_power *= m_A ;
    }
    //makes M matrices
    for (std::size_t i = 0; i < F; i++) {
        for (std::size_t j = 0; j < V; j++) {
            // upper triangular zero region
            if (j > i) {
                continue;
            }
            // Compute A^(i-j)
            Eigen::Matrix<double, n_states, n_states> A_term = 
                Eigen::Matrix<double, n_states, n_states>::Identity();
            
            //normal region: each control move is free
            //applies when prediction step i is within control horizon,
            //or when we're before the last free control move
            if (i < V || j < V - 1) {
                for (std::size_t k = 0; k < (i - j); k++) {
                    A_term *= m_A;
                }
                m_M.template block<r_states, m_inputs>(i * r_states, j * m_inputs) = 
                    m_C * A_term * m_B;
                m_M_xy.template block<2, m_inputs>(i * 2, j * m_inputs) = 
                    m_C_xy * A_term * m_B;
                m_M_omega.template block<2, m_inputs>(i * 2, j * m_inputs) = 
                    m_C_omega * A_term * m_B;
            }
            //held control region: last control move u_{V-1} is held constant
            //for all remaining prediction steps i >= V
            else {
                //A_bar = I + A + A² + ... + A^(i-V+1)
                //sums the effect of holding u_{V-1} over multiple steps
                Eigen::Matrix<double, n_states, n_states> A_bar = 
                    Eigen::Matrix<double, n_states, n_states>::Zero();
                Eigen::Matrix<double, n_states, n_states> A_sum = 
                    Eigen::Matrix<double, n_states, n_states>::Identity();
                for (std::size_t k = 0; k <= (i - V + 1); k++) {
                    if (k > 0) {
                        A_sum *= m_A;
                    }
                    A_bar += A_sum;
                }
                for (std::size_t k = 0; k < (V - 1 - j); k++) {
                    A_term *= m_A;
                }

                //A_term already has A^(i-j) where j = V-1, so i-j = i-(V-1)
                //we need A^(i-(V-1)) * A_bar * B for the held effect
                m_M.template block<r_states, m_inputs>(i * r_states, j * m_inputs) = 
                    m_C * A_term * A_bar * m_B;
                m_M_xy.template block<2, m_inputs>(i * 2, j * m_inputs) = 
                    m_C_xy * A_term * A_bar * m_B;
                m_M_omega.template block<2, m_inputs>(i * 2, j * m_inputs) = 
                    m_C_omega * A_term * A_bar * m_B;
            }
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
    m_b_delta.template segment<m_inputs>(m_inputs * V) -= u_prev; //upper bound
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
    m_b_z_xy.topRows(2 * F) = m_z_xy_max+ m_O_xy*m_x_hat;    
    m_b_z_xy.bottomRows(2 * F) = m_z_xy_max - m_O_xy*m_x_hat;
}

// motor speed constraint
template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildConstraintOmega() {
    //initialize b_z,omega
    m_b_z_omega.topRows(2 * F) = m_z_omega_max+ m_O_omega*m_x_hat;    
    m_b_z_omega.bottomRows(2 * F) = m_z_omega_max - m_O_omega*m_x_hat;
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
    m_s.noalias() = m_z_desired - m_O * m_x_hat;

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
}

//calls OSQP to solve constrained QP, extracts first optimal input
template<std::size_t V, std::size_t F>
void MPCController<V,F>::solveQP() {
    constexpr int N_VARS   = m_inputs * V;
    constexpr int N_CONSTR = 6*m_inputs*V + 8*F;

    //build P (upper triangular, OSQP expects symmetric half)
    Eigen::SparseMatrix<double> P_full   = m_P.sparseView();
    Eigen::SparseMatrix<double> P_upper  = P_full.triangularView<Eigen::Upper>();
    CscStorage P_csc = eigenToCSC(P_upper);

    //build A (constraint matrix)
    Eigen::SparseMatrix<double> G_sparse = m_G.sparseView();
    CscStorage A_csc = eigenToCSC(G_sparse);

    //build gradient q
    std::vector<OSQPFloat> q(N_VARS);
    for (int i = 0; i < N_VARS; i++) {
        q[i] = static_cast<OSQPFloat>(m_q(i));
    }

    //build lower bound
    std::vector<OSQPFloat> l(N_CONSTR);
    for (int i = 0; i < N_CONSTR; i++) {
        l[i] = -OSQP_INFTY;
    }

    //build upper bound
    std::vector<OSQPFloat> u(N_CONSTR);
    for (int i = 0; i < N_CONSTR; i++) {
        u[i] = static_cast<OSQPFloat>(m_b(i));
    }

    //set settings
    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.warm_starting = 0; //cold start each time
    settings.max_iter = 10000;  //set max iterations
    settings.polishing = 1; //refine solution
    settings.eps_abs = 1e-4; //set absolute tolerance
    settings.eps_rel = 1e-4; //set relative tolerance
    settings.verbose = 0; //make it silent

    //create solver instance
    OSQPSolver* solver = nullptr;
    OSQPInt setup_flag = osqp_setup(&solver, &P_csc.mat, q.data(), &A_csc.mat,
                                    l.data(), u.data(), N_CONSTR, N_VARS, &settings);
    if (setup_flag != 0) {
        throw std::runtime_error("OSQP setup failed");
    }

    //run optimizer
    if (solve_flag != 0 || solver->info->status_val != OSQP_SOLVED) {
        osqp_cleanup(solver);
        throw std::runtime_error("OSQP solve failed");
    }

    //extract full optimal sequence
    Eigen::Matrix<double, N_VARS, 1> u_star;
    for (int i = 0; i < N_VARS; i++) {
        u_star(i) = solver->solution->x[i];
    }

    //get first voltages
    Eigen::Matrix<double, m_inputs, 1> u_apply = u_star.template segment<m_inputs>(0);
    m_u_left  = u_apply(0);
    m_u_right = u_apply(1);

    //update u_prev
    m_u_prev  = u_apply;

    //free osqp memory
    osqp_cleanup(solver);
}

//outputs wheel voltages using MPC, taking in info from MPCUpdatePacket
template<std::size_t V, std::size_t F>
WheelVelocities MPCController<V, F>::compute(const Pose& currentPose, Eigen::Matrix<double, r_states * F, 1> z_desired, double omega_L, double omega_R, double V_battery, double I_total) {
    m_z_desired = z_desired;

    //linearize system
    linearize(currentPose, omega_L, omega_R);

    //discretize
    discretize();

    //build prediction matrices
    buildPredictionMatrices();

    //assemble constraints
    assembleConstraints(V_battery, I_total, m_u_prev);

    //build QP
    assembleQP();

    //solve QP
    solveQP();

    //return first optimal control as wheel voltages
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
    Eigen::Matrix<double, r_states * F, 1> z;

    for (std::size_t i = 0; i < F; i++) {
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
        WheelVelocities out = mpc.compute(
            pose,
            z_desired,
            packet.omega_L,
            packet.omega_R,
            packet.V_battery,
            packet.I_total
        );
    //pack optimal voltages on success
    response.V_left = static_cast<float>(out.left);
    response.V_right = static_cast<float>(out.right);
    }
    catch{
        //MPC failed: log error and send zero volts
        std::cerr << "MPC ERROR: " << e.what() << ", sending zero volts\n";
    }
    //send optimal voltages
    serial.send(SerialProtocol::PacketType::MPC_CONTROL, response);
    std::cerr << "sent V_L=" << response.V_left << " V_R=" << response.V_right<<  std::endl;
}

//explicit instantiation for V=F=15
template class MPCController<15, 15>;
