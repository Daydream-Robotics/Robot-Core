#include <iomanip>
#include <iostream>
#include "motion/control/mpc.hpp"
#include "utils/mathHelpers.hpp"
#include "config/constants.h"
#include <unsupported/Eigen/MatrixFunctions>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <array>
#include <vector>
#include <stdexcept>

template<class Drive, std::size_t V, std::size_t F>
MPCController<Drive, V, F>::Params::Params() {
    //geometry and motor model from the drive's own block in config/constants.h
    if constexpr (Drive::MODE == kinematics::DriveMode::DIFF) {
        geom.wheelRadius = DRIVE_WHEEL_DIAMETER_INCHES / 2.0; //wheel radius (in)
        geom.trackWidth = TRACK_WIDTH; //track width (in)
        a.fill(MPC_MOTOR_A); //motor time-constant
        b.fill(MPC_MOTOR_B); //motor gain constant
    }
    else if constexpr (Drive::MODE == kinematics::DriveMode::X) {
        geom.wheelRadius = XDRIVE_WHEEL_DIAMETER_INCHES / 2.0; //omni wheel radius (in)
        geom.rho = XDRIVE_RHO; //center to wheel (in)
        a.fill(XDRIVE_MOTOR_A);
        b.fill(XDRIVE_MOTOR_B);
    }
    else {
        geom.wheelRadius = HDRIVE_WHEEL_DIAMETER_INCHES / 2.0; //side wheel radius (in)
        geom.trackWidth = HDRIVE_TRACK_WIDTH; //side track width (in)
        geom.strafeRadius = HDRIVE_STRAFE_WHEEL_DIAMETER_INCHES / 2.0; //strafe wheel radius (in)
        geom.strafeOffset = HDRIVE_STRAFE_OFFSET; //strafe wheel forward offset (in)
        //group order L, R, strafe
        a = {HDRIVE_SIDE_MOTOR_A, HDRIVE_SIDE_MOTOR_A, HDRIVE_STRAFE_MOTOR_A};
        b = {HDRIVE_SIDE_MOTOR_B, HDRIVE_SIDE_MOTOR_B, HDRIVE_STRAFE_MOTOR_B};
    }
    omega_motor_max.fill(MPC_OMEGA_WHEEL_MAX); //max allowed wheel rad/s
    delta_u_max.fill(MPC_DELTA_U_MAX); //max positive change in voltage between steps
    delta_u_min.fill(MPC_DELTA_U_MIN); //max negative change in voltage between steps
    V_applied.fill(0.0); //starting voltage (stopped)
    h = MPC_SAMPLE_PERIOD; //sampling period (s)
    p_x = MPC_P_X; //output weight for x
    p_y = MPC_P_Y; //output weight for y
    p_theta = MPC_P_THETA; //output weight for theta
    q_u = MPC_Q_U; //input penalty q_u
    q_zero_multiplier = MPC_Q_ZERO_MULT; //multiplier for first Q_i
    q_final_multiplier = MPC_Q_FINAL_MULT; //multiplier for last Q_i
    p_final_multiplier = MPC_P_FINAL_MULT; //multiplier for last P_i
    V_max = MPC_V_MAX; //voltage max
    x_field_max = MPC_X_FIELD_MAX; //max allowed x position on field
    y_field_max = MPC_Y_FIELD_MAX; //max allowed y position on field
    command_delay = MPC_COMMAND_DELAY; //solve to applied (s)
}

//reference params from the controller's model, so the two can't disagree
//throws std::invalid_argument if the geometry is missing a value the drive's kinematics would divide by
template<class Drive, std::size_t F, class ControllerParams>
static typename MPCReference<Drive, F>::Params referenceParams(const ControllerParams& params) {
    if (!Drive::valid(params.geom)) {
        throw std::invalid_argument("MPC drive geometry is missing a value this drive needs");
    }
    typename MPCReference<Drive, F>::Params p;
    p.h = params.h;
    p.geom = params.geom;
    p.a = params.a;
    p.b = params.b;
    p.heading = params.heading;
    return p;
}

//constructor
template<class Drive, std::size_t V, std::size_t F>
MPCController<Drive, V, F>::MPCController(const Params& params)
    : m_params(params),
      m_reference(referenceParams<Drive, F>(params)) {
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

    //kinematics from the drivetrain traits
    const auto G = Drive::G(m_params.geom);
    const auto K = Drive::K(m_params.geom);
    for (std::size_t i = 0; i < m_inputs; i++) {
        for (std::size_t j = 0; j < 3; j++) {
            m_G_kin(i, j) = G[i][j];
            m_K_kin(j, i) = K[j][i];
        }
    }
    m_a_vec = Eigen::Map<const VecM>(m_params.a.data());
    m_b_vec = Eigen::Map<const VecM>(m_params.b.data());
    m_u_opt.setZero();

    //initialize C with an I_3 matrix in top left
    m_C.template leftCols<3>().setIdentity();
    //initialize B to have each group's gain on the bottom rows, diag(b)
    m_Bc.template bottomRows<m_inputs>() = m_b_vec.asDiagonal();
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
        m_W_four.template block<r_states, r_states>(r_states * i, r_states * i) = m_P_i.template topLeftCorner<r_states, r_states>();
    }
    m_W_four.template block<r_states, r_states>(r_states*(F-1), r_states*(F-1)) = m_params.p_final_multiplier * m_P_i.template topLeftCorner<r_states, r_states>();
    //initialize G_delta
    constexpr std::size_t NU = m_inputs * V;
    m_G_delta.template block<NU, NU>(0, 0) = -m_W_one;
    m_G_delta.template block<NU, NU>(NU, 0) = m_W_one;
    //initialize G_u
    m_G_u.template block<NU, NU>(0, 0) = -Eigen::Matrix<double, NU, NU>::Identity();
    m_G_u.template block<NU, NU>(NU, 0) = Eigen::Matrix<double, NU, NU>::Identity();
    //initialize G_b
    m_G_b=m_G_u;
    //initialize b_Delta
    fillDeltaBounds();
    //initialize b_u
    m_b_u.template topRows<m_inputs*V>().setConstant(m_params.V_max);
    m_b_u.template bottomRows<m_inputs*V>().setConstant(m_params.V_max);
    //initialize z_xy
    for (std::size_t i = 0; i<F; i++) {
        m_z_xy_max.template segment<2>(2*i) << m_params.x_field_max, m_params.y_field_max;
    }
    //initialize z_omega, one limit per group at every step
    for (std::size_t i = 0; i < F; i++) {
        for (std::size_t j = 0; j < m_inputs; j++) {
            m_z_omega_max(i * m_inputs + j) = m_params.omega_motor_max[j];
        }
    }
    //initialize C_xy, picks x and y
    m_C_xy.template leftCols<2>().setIdentity();
    //initialize C_omega, picks the group speeds
    m_C_omega.template rightCols<m_inputs>().setIdentity();
    //initialize u_prev
    m_u_prev = Eigen::Map<const VecM>(m_params.V_applied.data());
}

//per-group delta-u bounds for every control step, before the first step is shifted by u_prev
template<class Drive, std::size_t V, std::size_t F>
void MPCController<Drive, V, F>::fillDeltaBounds() {
    for (std::size_t k = 0; k < V; k++) {
        for (std::size_t j = 0; j < m_inputs; j++) {
            m_b_delta(k * m_inputs + j) = -m_params.delta_u_min[j];
            m_b_delta(m_inputs * V + k * m_inputs + j) = m_params.delta_u_max[j];
        }
    }
}

//build per-stage LTV models: linearize + discretize about each ref state
template<class Drive, std::size_t V, std::size_t F>
void MPCController<Drive, V, F>::buildStageModels() {
    Eigen::Matrix<double, n_states + m_inputs, n_states + m_inputs> m_exp;

    for (std::size_t i = 0; i < F; i++) {
        //ref heading and group speeds for the stage
        double theta = m_x_ref(2, i);
        VecM w_ref = m_x_ref.col(i).template tail<m_inputs>();
        //body twist at the stage ref (vx forward, vy left)
        Eigen::Vector3d xi = m_K_kin * w_ref;
        double vx = xi(0);
        double vy = xi(1);
        //diff drive only: floor vx so the theta col remains nonzero (same as flooring omega_L + omega_R at 2 rad/s)
        if constexpr (!Drive::HOLONOMIC) {
            constexpr double OMEGA_SUM_EPS = 2.0;
            const double V_EPS = m_params.geom.wheelRadius * OMEGA_SUM_EPS / 2.0;
            if (std::abs(vx) < V_EPS) {
                vx = (vx >= 0.0) ? V_EPS : -V_EPS;
            }
        }
        //cont Jacobian at stage-i ref
        //x_dot = vx sin(theta) - vy cos(theta), y_dot = vx cos(theta) + vy sin(theta), theta_dot = -w
        double c = std::cos(theta);
        double s = std::sin(theta);
        //body (forward, left) to the pose frame
        Eigen::Matrix2d P;
        P << s, -c,
             c,  s;
        m_Ac.setZero();
        m_Ac(0, 2) =  c * vx + s * vy;
        m_Ac(1, 2) = -s * vx + c * vy;
        m_Ac.template block<2, m_inputs>(0, 3) = P * m_K_kin.template topRows<2>();
        //pose theta turns clockwise, the K rotation row is counterclockwise
        m_Ac.template block<1, m_inputs>(2, 3) = -m_K_kin.row(2);
        m_Ac.template bottomRightCorner<m_inputs, m_inputs>() = (-m_a_vec).asDiagonal();
        //ZOH discretize the curr stage model
        m_exp.setZero();
        m_exp.template block<n_states, n_states>(0, 0) = m_Ac;
        m_exp.template block<n_states, m_inputs>(0, n_states) = m_Bc;
        m_exp *= m_params.h;
        auto m_d = m_exp.exp().eval();
        m_A_k[i] = m_d.template block<n_states, n_states>(0, 0);
        m_B_k[i] = m_d.template block<n_states, m_inputs>(0, n_states);

        //affine term  d_k = x_ref[k+1] - A_k*x_ref[k] - B_k*u_ref[k]
        m_d_k[i] = m_x_ref.col(i + 1)
                 - m_A_k[i] * m_x_ref.col(i)
                 - m_B_k[i] * m_u_ref.col(i);
    }
}

//reconstruct full reference states (x, y, theta, group speeds) and
//feedforward voltages from the F+1 reference poses
template<class Drive, std::size_t V, std::size_t F>
void MPCController<Drive, V, F>::buildReferenceStates() {
    const double h = m_params.h;

    for (std::size_t i = 0; i < F; i++) {
        //consecutive ref poses i and i+1
        double x_zero  = m_z_desired(3*i + 0);
        double y_zero  = m_z_desired(3*i + 1);
        double theta_zero  = m_z_desired(3*i + 2);
        double x_one  = m_z_desired(3*(i+1) + 0);
        double y_one  = m_z_desired(3*(i+1) + 1);
        double theta_one = m_z_desired(3*(i+1) + 2);

        double c = std::cos(theta_zero);
        double s = std::sin(theta_zero);
        //signed forward speed: displacement projected onto heading (heading direction is (sin, cos))
        double vx = ((x_one - x_zero) * s + (y_one - y_zero) * c) / h;
        //signed left speed: displacement projected onto the robot's left (-cos, sin)
        double vy = (-(x_one - x_zero) * c + (y_one - y_zero) * s) / h;
        //heading rate from finite difference (positive turns from +y toward +x), negated to counterclockwise for G
        double w = -utils::wrapAngle(theta_one - theta_zero) / h;

        //inverse kinematics: group speeds from the body twist, diff drive drops vy since its G column is zero
        VecM w_ref = m_G_kin * Eigen::Vector3d(vx, vy, w);

        m_x_ref.col(i) << x_zero, y_zero, theta_zero, w_ref;
    }
    //terminal reference state: last pose, hold last group speeds
    m_x_ref.col(F) << m_z_desired(3*F + 0), m_z_desired(3*F + 1), m_z_desired(3*F + 2),
                      m_x_ref.col(F - 1).template tail<m_inputs>();
    //feedforward voltage that takes each group from stage i's reference speed to stage i+1's in one step,
    //from the motor's exact discrete step w[i+1] = e^(-ah) w[i] + (1 - e^(-ah)) (b/a) u[i], per group
    const VecM decay = (-m_a_vec.array() * h).exp().matrix();
    const VecM gain = ((1.0 - decay.array()) * m_b_vec.array() / m_a_vec.array()).matrix();
    for (std::size_t i = 0; i < F; i++) {
        m_u_ref.col(i) = ((m_x_ref.col(i + 1).template tail<m_inputs>().array()
                           - decay.array() * m_x_ref.col(i).template tail<m_inputs>().array()) / gain.array()).matrix();
    }
}

//build LTV pred. matrices over the horizon
template<class Drive, std::size_t V, std::size_t F>
void MPCController<Drive, V, F>::buildPredictionMatrices() {
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
        m_O_omega.template block<m_inputs, n_states>(i * m_inputs, 0) = m_C_omega * m_Phi;

        //affine-response rows: D = C * w
        m_D_z.template segment<r_states>(i * r_states) = m_C * m_w;
        m_D_xy.template segment<2>(i * 2) = m_C_xy * m_w;
        m_D_omega.template segment<m_inputs>(i * m_inputs) = m_C_omega * m_w;

        //forced-response rows: M[i][j] = C * E[j] for every active input
        for (std::size_t j = 0; j <= std::min(i, V - 1); j++) {
            m_M.template block<r_states, m_inputs>(i * r_states, j * m_inputs) = m_C * m_E[j];
            m_M_xy.template block<2, m_inputs>(i * 2, j * m_inputs) = m_C_xy * m_E[j];
            m_M_omega.template block<m_inputs, m_inputs>(i * m_inputs, j * m_inputs) = m_C_omega * m_E[j];
        }
    }
}

//assemble full G, b: stacks all inequality constraints into single matrix G and vector b
template<class Drive, std::size_t V, std::size_t F>
void MPCController<Drive, V, F>::assembleConstraints(double V_batt, const Eigen::Matrix<double, m_inputs, 1> u_prev) {
    //rebuild time-varying constr
    buildConstraintDelta(u_prev);
    buildConstraintBattery(V_batt);
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
    m_G.middleRows(row, 2 * m_inputs * F) = m_G_z_omega;
    m_b.segment(row, 2 * m_inputs * F) = m_b_z_omega;
}

//delta u constraint: limits how fast voltage can change: |u_k - u_{k-1}| <= delta_u_max
template<class Drive, std::size_t V, std::size_t F>
void MPCController<Drive, V, F>::buildConstraintDelta(const Eigen::Matrix<double, m_inputs, 1> u_prev) {
    // base limits from params
    fillDeltaBounds();
    //adjust first block for actual previous control
    //lower bound row: -(u_0 - u_prev) <= -delta_u_min, so -u_0 <= -delta_u_min - u_prev
    m_b_delta.template segment<m_inputs>(0) -= u_prev;
    //upper bound row: u_0 - u_prev <= delta_u_max, so u_0 <= delta_u_max + u_prev
    m_b_delta.template segment<m_inputs>(m_inputs * V) += u_prev;
}

//battery voltage constraint: the brain measures the battery under load, so that voltage (sag included) is what the motors can get
template<class Drive, std::size_t V, std::size_t F>
void MPCController<Drive, V, F>::buildConstraintBattery(double V_battery) {
    double u_b_max = V_battery;
    //symmetric limit for every group, at all steps
    m_b_b.template topRows<m_inputs*V>().setConstant(u_b_max);
    m_b_b.template bottomRows<m_inputs*V>().setConstant(u_b_max);
}

// field position constraint as soft field boundaries: computed as output constraint on predicted positions
template<class Drive, std::size_t V, std::size_t F>
void MPCController<Drive, V, F>::buildConstraintPosition() {
    m_b_z_xy.topRows(2 * F) = m_z_xy_max + (m_O_xy * m_x_hat + m_D_xy);
    m_b_z_xy.bottomRows(2 * F) = m_z_xy_max - (m_O_xy * m_x_hat + m_D_xy);
    m_G_z_xy.topRows(2 * F) = -m_M_xy;
    m_G_z_xy.bottomRows(2 * F) =  m_M_xy;
}

// motor speed constraint
template<class Drive, std::size_t V, std::size_t F>
void MPCController<Drive, V, F>::buildConstraintOmega() {
    //initialize b_z,omega
    m_b_z_omega.topRows(m_inputs * F) = m_z_omega_max + (m_O_omega * m_x_hat + m_D_omega);
    m_b_z_omega.bottomRows(m_inputs * F) = m_z_omega_max - (m_O_omega * m_x_hat + m_D_omega);
    m_G_z_omega.topRows(m_inputs * F) = -m_M_omega;
    m_G_z_omega.bottomRows(m_inputs * F) =  m_M_omega;
}


//build QP matrices: forms condensed QP:  min_U  1/2 U^T*P*U + q^T*U: P = M^T*W_4*M + W_3; q = -M^T*W_4*s: s = z_des - O*x_hat
template<class Drive, std::size_t V, std::size_t F>
void MPCController<Drive, V, F>::assembleQP() {
    // tracking error: s = z_desired - free response
    m_s.noalias() = m_z_desired.template segment<r_states * F>(r_states) - m_O * m_x_hat - m_D_z;
    //wrap every heading error to (-pi, pi]
    for (std::size_t i = 0; i < F; i++) {
        m_s(i * r_states + 2) = utils::wrapAngle(m_s(i * r_states + 2));
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
template<class Drive, std::size_t V, std::size_t F>
template<typename Derived>
void MPCController<Drive, V, F>::fillCscDense(CscStorage& out, const Eigen::MatrixBase<Derived>& m_M, bool upperOnly) {
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
template<class Drive, std::size_t V, std::size_t F>
void MPCController<Drive, V, F>::initCscDense(CscStorage& out, int rows, int cols, bool upperOnly) {
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
template<class Drive, std::size_t V, std::size_t F>
void MPCController<Drive, V, F>::solveQP() {
    constexpr int N_VARS = m_inputs * V;
    constexpr int N_CONSTR = n_constr;

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

    //first stage optimal input (u_0), one voltage per group
    const VecM u0 = Eigen::Map<const VecM>(m_solver->solution->x);

    // OSQP_SOLVED_INACCURATE can return a stage-0 input that violates the hard delta-u rate limit so we treat that the same as a failed solve
    // numerical tolerance above the hard limit
    constexpr double kDeltaSlack = 2;
    for (std::size_t j = 0; j < m_inputs; j++) {
        double d = u0(j) - m_u_prev(j);
        if (d > m_params.delta_u_max[j] + kDeltaSlack || d < m_params.delta_u_min[j] - kDeltaSlack) {
            std::cerr << "OSQP solution violates delta-u limit (group " << j << " d=" << d << ")\n";
            throw std::runtime_error("OSQP solution violates rate limit");
        }
    }

    //store applied input for next cycle's delta-u constraint and cost
    m_u_opt = u0;
    m_u_prev = u0;

    //warm start next solve: shift solution one stage left and repeat last stage
    m_x_ws.resize(N_VARS);
    for (int i = 0; i + m_inputs < N_VARS; i++) {
        m_x_ws[i] = m_solver->solution->x[i + m_inputs];
    }
    for (int i = N_VARS - m_inputs; i < N_VARS; i++) {
        m_x_ws[i] = m_solver->solution->x[i];
    }
    osqp_warm_start(m_solver, m_x_ws.data(), OSQP_NULL);
}

//outputs group voltages using MPC
template<class Drive, std::size_t V, std::size_t F>
typename MPCController<Drive, V, F>::VecM MPCController<Drive, V, F>::compute(const Pose& currentPose, const Eigen::Matrix<double, r_states*(F+1), 1>& z_ref, const VecM& omega, double V_battery) {
     //store reference trajectory and current state estimate
    m_z_ref = z_ref;
    m_z_desired = z_ref;
    m_x_hat << currentPose.x, currentPose.y, currentPose.theta, omega;

    try {
        //reject NaN/inf inputs before they reach OSQP
        if (!std::isfinite(currentPose.x) || !std::isfinite(currentPose.y) || !std::isfinite(currentPose.theta) ||
            !omega.allFinite() || !std::isfinite(V_battery) ||
            !z_ref.allFinite() || !m_u_prev.allFinite()) {
            throw std::runtime_error("MPC input is not finite");
        }
        buildReferenceStates();
        buildStageModels();
        buildPredictionMatrices();
        assembleConstraints(V_battery, m_u_prev);
        assembleQP();
        solveQP();
    }
    catch (...) {
        //failed solve: caller sends 0V (failsafe), so record 0V as applied for the next delta-u limit
        m_u_opt.setZero();
        m_u_prev.setZero();
        //drop the OSQP workspace so a bad solve can't poison later ones; next solve sets up fresh
        if (m_solver) {
            osqp_cleanup(m_solver);
            m_solver = nullptr;
        }
        throw;
    }
    return m_u_opt;
}

//path following: build the reference from the closest sample on, then solve
template<class Drive, std::size_t V, std::size_t F>
DriveCommand MPCController<Drive, V, F>::compute(const Pose& currentPose, const DriveState& drive, const ALS_Path& als_path, std::size_t& closestSampleIdx, PathFlag flag) {
    DriveCommand out;
    out.input = ControlMode::INPUT_VOLTAGE;
    try {
        const VecM omega = Eigen::Map<const VecM>(drive.omega.data());
        auto z = m_reference.build(currentPose, als_path, closestSampleIdx, flag == PathFlag::REVERSE, omega);
        const VecM u = compute(currentPose, z, omega, drive.batteryVolts);
        //V -> mV
        for (std::size_t j = 0; j < m_inputs; j++) {
            out.value[j] = u(j) * 1000.0;
        }
    }
    catch (const std::exception& e) {
        //failed solve: 0 V this step (compute already recorded 0 V as applied)
        std::cerr << "[MPC] " << e.what() << ", sending zero volts\n";
        out.value.fill(0.0);
    }
    return out;
}

//delay compensation: the brain's wheel speeds are drive.age old and the command applies command_delay after the solve
template<class Drive, std::size_t V, std::size_t F>
void MPCController<Drive, V, F>::predict(Pose& pose, DriveState& drive) const {
    VecM omega = Eigen::Map<const VecM>(drive.omega.data());
    predictState(pose, omega, drive.age, drive.batteryVolts);
    for (std::size_t j = 0; j < m_inputs; j++) {
        drive.omega[j] = omega(j);
    }
    drive.age = 0.0;
}

//delay compensation for any drive: omega is age old and the command applies command_delay after the solve
template<class Drive, std::size_t V, std::size_t F>
void MPCController<Drive, V, F>::predictState(Pose& pose, VecM& omega, double age, double batteryVolts) const {
    //longest measurement age to predict over (s)
    constexpr double MAX_AGE = 0.05;
    //voltage the brain is applying: the last one sent, which the battery caps
    const double v_cap = batteryVolts > 0.0 ? std::min(m_params.V_max, batteryVolts) : m_params.V_max;
    const VecM u = m_u_prev.cwiseMax(-v_cap).cwiseMin(v_cap);
    //exact motor step over dt at a constant voltage, per group
    auto motor = [this, &u](const VecM& w, double dt) -> VecM {
        const VecM decay = (-m_a_vec.array() * dt).exp().matrix();
        return (decay.array() * w.array() + (1.0 - decay.array()) * (m_b_vec.array() / m_a_vec.array()) * u.array()).matrix();
    };
    //group speeds now, then when the command applies
    const double dt = m_params.command_delay;
    const VecM w_now = motor(omega, std::clamp(age, 0.0, MAX_AGE));
    const VecM w_end = motor(w_now, dt);
    //pose over the command delay at the average of the two speeds, heading at the midpoint
    const Eigen::Vector3d xi = m_K_kin * (0.5 * (w_now + w_end));
    const double vx = xi(0);
    const double vy = xi(1);
    //pose theta turns clockwise, the K rotation row is counterclockwise
    const double theta_dot = -xi(2);
    pose.theta += theta_dot * dt / 2.0;
    pose.x += (vx * std::sin(pose.theta) - vy * std::cos(pose.theta)) * dt;
    pose.y += (vx * std::cos(pose.theta) + vy * std::sin(pose.theta)) * dt;
    pose.theta += theta_dot * dt / 2.0;
    omega = w_end;
}

//clears all dynamic data for fresh start (ie. new path segment)
template<class Drive, std::size_t V, std::size_t F>
void MPCController<Drive, V, F>::reset() {
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
    m_u_opt.setZero();
    //back to the constructor's starting voltages
    m_u_prev = Eigen::Map<const VecM>(m_params.V_applied.data());
    m_reference.reset();
    //drop the OSQP workspace (warm start, adapted rho) so the next solve sets up fresh
    if (m_solver) {
        osqp_cleanup(m_solver);
        m_solver = nullptr;
    }
}


template<class Drive, std::size_t V, std::size_t F>
MPCController<Drive, V, F>::~MPCController() {
    if (m_solver) {
        osqp_cleanup(m_solver);
    }
}

//explicit instantiation for every drive at the horizons in config/constants.h
template class MPCController<kinematics::DiffDrive, MPC_CONTROL_HORIZON, MPC_PREDICTION_HORIZON>;
template class MPCController<kinematics::XDrive, MPC_CONTROL_HORIZON, MPC_PREDICTION_HORIZON>;
template class MPCController<kinematics::HDrive, MPC_CONTROL_HORIZON, MPC_PREDICTION_HORIZON>;


