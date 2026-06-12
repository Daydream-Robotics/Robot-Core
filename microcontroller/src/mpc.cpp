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

inline double wrapAngle(double angle) {
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}


template<std::size_t V, std::size_t F>
void MPCController<V, F>::linearize(const Pose& x_hat, double omega_L, double omega_R) {
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

template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildPredictionMatrices() {
    //makes matrix to hold A to a power
    Eigen::Matrix<double, n_states, n_states> m_A_power = m_A;
    //makes O matrix
    for (std::size_t i = 0; i<F; i++){
        m_O.template block<r_states, n_states> (i*r_states, 0) = m_C * m_A_power;
        m_O_xy.template block<2, n_states>(i*2, 0) = m_C_xy * m_A_power;
        m_O_omega.template block<2, n_states>(i*2, 0) = m_C_omega * m_A_power;
        m_A_power *= m_A ;
    }
    //initialize M
    for (std::size_t i = 0; i < F; i++) {
        for (std::size_t j = 0; j < V; j++) {
            // upper triangular zero region
            if (j > i) {
                continue;
            }
            Eigen::Matrix<double, n_states, n_states> A_term = 
                Eigen::Matrix<double, n_states, n_states>::Identity();
            
            // normal region
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
            // held control region
            else {
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

template<std::size_t V, std::size_t F>
void MPCController<V, F>::assembleConstraints(double V_batt, double I_total, const Eigen::Matrix<double, m_inputs, 1> u_prev) {
    buildConstraintDelta(u_prev);
    buildConstraintU();
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

template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildConstraintDelta(const Eigen::Matrix<double, m_inputs, 1> u_prev) {
    //add prev u to b_Dleta
    m_b_b.template segment<m_inputs>(0) += u_prev;
    m_b_b.template segment<m_inputs>(m_inputs * V) -= u_prev;
}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildConstraintBattery(double V_battery, double I_total) {
    //initialize max input due to battery sag
    double u_b_max = V_battery - m_params.R_internal*I_total;
    //initialize b_battery 
    m_b_delta.template topRows<m_inputs*V>().setConstant(u_b_max);
    m_b_delta.template bottomRows<m_inputs*V>().setConstant(u_b_max);

}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildConstraintPosition() {
    //initialize b_z,xy
    m_b_z_xy.topRows(2 * F) = m_z_xy_max+ m_O_xy*m_x_hat;    
    m_b_z_xy.bottomRows(2 * F) = m_z_xy_max - m_O_xy*m_x_hat;
}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::buildConstraintOmega() {
    //initialize b_z,omega
    m_b_z_omega.topRows(2 * F) = m_z_omega_max+ m_O_omega*m_x_hat;    
    m_b_z_omega.bottomRows(2 * F) = m_z_omega_max - m_O_omega*m_x_hat;
}

// template<std::size_t V, std::size_t F>
// InterpSample MPCController<V, F>::sampleAtArcLength(const std::vector<Sample>& samples, double sQuery) {
//     InterpSample result;
//     if (samples.empty()) {
//         return result;
//     }
//     if (sQuery <= samples.front().s) {
//         result.x = samples.front().x;
//         result.y = samples.front().y;
//         result.theta = samples.front().theta;
//         result.v = samples.front().v;
//         return result;
//     }
//     if (sQuery >= samples.back().s) {
//         result.x = samples.back().x;
//         result.y = samples.back().y;
//         result.theta = samples.back().theta;
//         result.v = samples.back().v;
//         return result;
//     }

//     std::size_t left = 0;
//     std::size_t right = samples.size()-1;

//     while (left+1 < right) {
//         std::size_t mid = left + (right-left)/2;
//         if (samples[mid].s <= sQuery) {
//             left = mid;
//         }
//         else {
//             right = mid;
//         }
//     }
//     const Sample& a = samples[left];
//     const Sample& b = samples[right];

//     double ds = b.s - a.s;
//     double alpha = (std::abs(ds) < 1e-12) ? 0.0 : (sQuery - a.s) /ds;
//     double dtheta = wrapAngle(b.heading-a.heading);

//     result.x = a.x + alpha * (b.x-a.x);
//     result.y = a.y + alpha * (b.y-a.y);
//     result.heading = wrapAngle(a.heading + alpha * dtheta);
//     result.v = a.v + alpha * (b.v-a.v);

//     return result;
// }

// template<std::size_t V, std::size_t F>
// void MPCController<V, F>::buildZDesired(const ALS_Path& als_path, std::size_t closestSampleIdx) {
//     const std::vector<Sample>& samples = als_path.getSamples();

//     if (samples.empty() || !als_path.isValid()) {
//         return;
//     }
//     if (closestSampleIdx >= samples.size()) {
//         closestSampleIdx = samples.size()-1;
//     }

//     double s_prev = samples[closestSampleIdx].s;

//     for (std::size_t i = 0; i < F; i++) {
//         InterpSample curr = sampleAtArcLength(samples, s_prev);
//         double v_path = std::max(curr.v, 0.0);
//         s_prev += v_path * m_params.h;

//         s_prev = std::min(s_prev, als_path.getTotalLength());

//         InterpSample ref = sampleAtArcLength(samples, s_prev);

//         const int base = i * r_states;
//         m_z_desired(base+0) = ref.x;
//         m_z_desired(base+1) = ref.y;
//         m_z_desired(base+2) = ref.theta;
//     }
// }

// template<std::size_t V, std::size_t F>
// void MPCController<V, F>::buildZDesiredFromPacket(const MPCPacket& packet) {
//     for (std::size_t i = 0; i < F; i++) {
//         const float x = packet.data[i * 3 + 0];
//         const float y = packet.data[i * 3 + 1];
//         const float t = packet.data[i * 3 + 2];

//         const int base = i * r_states;
//         m_z_desired(base + 0) = x;
//         m_z_desired(base + 1) = y;
//         m_z_desired(base + 2) = t;
//     }
// }

template<std::size_t V, std::size_t F>
void MPCController<V, F>::assembleQP() {
    m_s.noalias() = m_z_desired - m_O * m_x_hat;
    Eigen::Matrix<double, r_states*F, m_inputs*V> m_W_four_M;
    m_W_four_M.noalias() = m_W_four * m_M;
    m_P.noalias() = m_M.transpose() * m_W_four_M + m_W_three;
    Eigen::Matrix<double, r_states*F, 1> m_W_four_s;
    m_W_four_s.noalias() = m_W_four * m_s;
    m_q.noalias() = m_M.transpose() * m_W_four_s;  
}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::solveQP() {
    //convert dens matrices to sparse
    Eigen::SparseMatrix<double> P_sparse = m_P.sparseView();
    Eigen::SparseMatrix<double> G_sparse = m_G.sparseView();

    Eigen::Matrix<double, m_inputs * V, 1> q_osqp = -m_q;

    auto lower_bound;

    //lower bound
    lower_bound.setConstant(-OSQP_INFTY);

    //upper bound
    auto upper_bound = m_b;

    //create solver
    OsqpEigen::Solver solver;

    //set sttings
    solver.settings()->setWarmStart(true);
    solver.settings()->setVerbosity(false);

    //set dimensions
    solver.data()->setNumberOfVariables(m_inputs * V);
    solver.data()->setNumberOfConstraints(m_b.rows());

    //pass Hessian
    if(!solver.data()->setHessianMatrix(P_sparse)) {
        throw std::runtime_error("Failed to set Hessian");
    }
    //pass gradient
    if (!solver.data()->setGradient(q_osqp)) {
        throw std::runtime_error("Failed to set gradient");
    }
    // pass constraint matrix
    if (!solver.data()->setLinearConstraintsMatrix(G_sparse)) {
        throw std::runtime_error("Failed to set constraint matrix");
    }
    // pass lower bound
    if (!solver.data()->setLowerBound(lower_bound)) {
        throw std::runtime_error("Failed to set lower bound");
    }
    // pass upper bound
    if (!solver.data()->setUpperBound(upper_bound)) {
        throw std::runtime_error("Failed to set upper bound");
    }
    // initialize solver
    if (!solver.initSolver()) {
        throw std::runtime_error("Failed to initialize OSQP");
    }

    //solve
    OsqpEigen::ErrorExitFlag status = solver.solveProblem();
    // check solve status
    if (status != OsqpEigen::ErrorExitFlag::NoError) {
        throw std::runtime_error("OSQP solve failed");
    }
    // optimal solution:
    // [u0_L, u0_R, u1_L, u1_R, ...]^T
    Eigen::Matrix<double, m_inputs * V, 1> u_star = solver.getSolution();
    // apply first control only
    Eigen::Matrix<double, m_inputs, 1> u_apply = u_star.template segment<m_inputs>(0);
    // store outputs
    m_u_left = u_apply(0);
    m_u_right = u_apply(1);
    //set u_prev 
    m_u_prev = u_apply;
}

template<std::size_t V, std::size_t F>
void MPCController<V, F>::reset() {
    m_x_hat.setZero();

    m_z_desired.setZero();
    m_s.setZero();

    m_q.setZero();
    m_P.setZero();

    m_b.setZero();

    m_b_delta.setZero();
    m_b_u.setZero();
    m_b_b.setZero();
    m_b_z_xy.setZero();
    m_b_z_omega.setZero();

    m_u_left = 0.0;
    m_u_right = 0.0;
}

template<std::size_t V, std::size_t F>
WheelVelocities MPCController<V, F>::compute(const Pose& currentPose, Eigen::Matrix<double, r_states * F, 1> z_desired, double omega_L, double omega_R, double V_battery, double I_total) {
    m_z_desired = z_desired;
    // 1. linearize system
    linearize(currentPose, omega_L, omega_R);

    // 2. discretize
    discretize();

    // 3. build prediction matrices
    buildPredictionMatrices();

    // 5. assemble constraints
    assembleConstraints(V_battery, I_total, m_u_prev);

    // 6. build QP
    assembleQP();

    // 7. solve QP
    solveQP();

    return WheelVelocities{m_u_left, m_u_right, ControlMode::INPUT_VOLTAGE};
}


template<std::size_t V, std::size_t F>
Eigen::Matrix<double, MPCController<V, F>::r_states * F, 1> MPCController<V, F>::unpackZDesired(const float* z_raw) {
    Eigen::Matrix<double, r_states * F, 1> z;

    for (std::size_t i = 0; i < F; i++) {
        const std::size_t base = i * 3;

        z(base + 0) = static_cast<double>(z_raw[base + 0]);
        z(base + 1) = static_cast<double>(z_raw[base + 1]);
        z(base + 2) = static_cast<double>(z_raw[base + 2]);
    }

    return z;
}
template<std::size_t V, std::size_t F>
void MPCController<V, F>::MPCControl(SerialProtocol& serial, MPCController& mpc) {

    std::optional<MPCUpdatePacket> packet_opt = serial.receive<MPCUpdatePacket>(SerialProtocol::PacketType::MPC_UPDATE);
    if(!packet_opt.has_value()) {
        return;
    }

    const MPCUpdatePacket& packet = *packet_opt;

    Pose pose{
        packet.pose_x,
        packet.pose_y,
        packet.pose_theta
    };

    
    auto z_desired = unpackZDesired(packet.z_desired);
    WheelVelocities out = mpc.compute(
        pose,
        z_desired,
        packet.omega_L,
        packet.omega_R,
        packet.V_battery,
        packet.I_total
    );

    MPCControlPacket response{};
    response.V_left = static_cast<float>(out.left);
    response.V_right = static_cast<float>(out.right);

    serial.send(SerialProtocol::PacketType::MPC_CONTROL, response);
}
template class MPCController<40, 40>;



/*
while (true) {

    MPCUpdatePacket req;
    fread(&req, sizeof(req), 1, stdin);

    // run MPC solver using:
    // req.pose_x, req.z_desired, etc...

    MPCControlPacket out;
    out.V_left = solve_left();
    out.V_right = solve_right();

    fwrite(&out, sizeof(out), 1, stdout);
    fflush(stdout);
}
*/