//Inclusions
#include "main.h"
#include "control/sensors.hpp"
#include "control/odom.hpp"
#include "control/range.hpp"
#include "control/mcl.hpp"
#include "control/rng.hpp"

//Created namespace mcl
namespace mcl {
    //field boundaries in inches
    double const X_MIN = -70;
    double const X_MAX = 70;
    double const Y_MIN = -70;
    double const Y_MAX = 70;
    //sensor noise standard deviation
    double const SIGMA_M = 0.25;
    //precomputed constants for gaussian calculation
    const double INV_2_SIGMA2 = 1.0 / (2.0 * SIGMA_M * SIGMA_M);
    const double LOG_NORM = std::log(2.0 * M_PI * SIGMA_M * SIGMA_M);
    //random number generator initialized with current time
    rng::xoshiro128plus rng_gen(pros::millis());

    //number of particles for MCL
    constexpr std::size_t N = 1000;
    //threshold for resampling (half of N)
    constexpr std::size_t N_T = N / 2;
    //precomputed 1/N for efficiency
    constexpr double ONE_N = 1.0 / static_cast<double>(N);
    // constexpr double MCL_SETPOSE_TRUST_THRESHOLD = 0.70;
    // constexpr std::uint32_t MCL_SETPOSE_COOLDOWN_MS = 50;
    constexpr std::uint32_t MCL_ESTIMATE_PRINT_INTERVAL_MS = 1000;
    //estimated robot position from MCL
    robot_position est_pos = {0,0,0};
    int nan_debug_prints = 0;
    bool range_system_initialized = false;
    bool mcl_initialized = false;
    pros::task_t odom_task_handle = nullptr;
    pros::task_t mcl_update_task_handle = nullptr;
    // std::uint32_t last_pose_correction_ms = 0;
    std::uint32_t last_estimate_print_ms = 0;
    //previous odometry position for delta calculation
    odom::odom_position prev_odom_pos;
    //change in odometry position since last update
    odom::odom_position delta_odom_pos;

    //motion noise covariance matrix
    constexpr Covariance EPSILON = {
        0.5 * 0.5, 0.0, 0.0, // x_x, x_y, x_theta
        0.0, 0.5 * 0.5, 0.0, // y_x, y_y, y_theta
        0.0, 0.0, 0.02 * 0.02 // theta_x, theta_y, theta_theta
    };
    constexpr double SIGMA_X = 0.5;
    constexpr double SIGMA_Y = 0.5;

    //vector storing all particles
    std::vector<Particle> particles;

    //mutex for thread-safe position updates
    pros::Mutex pos_update_mutex;

    //wraps angle to range [-pi, pi]
    inline double wrap_to_pi(double a) {
        if (a > M_PI)  a -= 2.0 * M_PI;
        if (a <= -M_PI) a += 2.0 * M_PI;
        return a;
    }

    //subtracts two odometry positions with angle wrapping
    inline odom::odom_position operator-(const odom::odom_position& a,
        const odom::odom_position& b) {
        return {
            a.x - b.x,
            a.y - b.y,
            wrap_to_pi(a.theta - b.theta)
        };
    }

    //calculates log of gaussian probability
    inline double log_gaussian(double z, double mu) {
        double r = z - mu;
        return -(r * r) * INV_2_SIGMA2 - 0.5 * LOG_NORM;
    }

    inline double clamp01(double value) {
        if (value < 0.0) return 0.0;
        if (value > 1.0) return 1.0;
        return value;
    }

    double chassis_delta_score(const robot_position& estimate, const lemlib::Pose& current_pose) {
        const double dx = estimate.x - current_pose.x;
        const double dy = estimate.y - current_pose.y;
        const double position_delta = std::sqrt(dx * dx + dy * dy);

        return clamp01((18.0 - position_delta) / 18.0);
    }

    bool task_is_active(pros::task_t task_handle) {
        if (task_handle == nullptr) return false;

        const auto state = pros::c::task_get_state(task_handle);
        return state != pros::E_TASK_STATE_DELETED && state != pros::E_TASK_STATE_INVALID;
    }

    //fast approximation of exponential function
    inline float fast_exp(float x) {
        x = 1.0f + x / 256.0f;
        x *= x; x *= x; x *= x; x *= x;
        x *= x; x *= x; x *= x; x *= x;
        return x;
    }

    void set_uniform_weights() {
        if (particles.empty()) return;

        const double inverse_N = 1.0 / particles.size();
        for (auto& element : particles) {
            element.w = 1.0;
            element.W = inverse_N;
            element.log_w = 0.0;
        }
    }

    double particle_concentration_score(const robot_position& estimate) {
        if (particles.empty()) return 0.0;

        double total_weight = 0.0;
        double weighted_sq_radius = 0.0;

        for (const auto& particle : particles) {
            if (!std::isfinite(particle.W)) continue;

            const double dx = particle.x - estimate.x;
            const double dy = particle.y - estimate.y;

            weighted_sq_radius += particle.W * (dx * dx + dy * dy);
            total_weight += particle.W;
        }

        if (!std::isfinite(total_weight) || total_weight <= 0.0) return 0.0;

        const double rms_radius = std::sqrt(weighted_sq_radius / total_weight);
        return clamp01((18.0 - rms_radius) / 18.0);
    }

    double trust_mcl_now(const robot_position& estimate, const lemlib::Pose& current_pose) {
        if (!mcl_initialized || particles.empty()) return 0.0;
        if (!std::isfinite(estimate.x) || !std::isfinite(estimate.y) || !std::isfinite(estimate.theta)) {
            return 0.0;
        }

        Particle estimate_particle = {
            estimate.x,
            estimate.y,
            estimate.theta,
            1.0,
            1.0,
            0.0
        };

        const sensors::range_readings expected = range::raycast(estimate_particle);
        double total_error = 0.0;
        int valid_sensor_count = 0;

        for (int i = 0; i < 4; i++) {
            const double actual = sensors::distance_readings.direction[i];
            const double predicted = expected.direction[i];

            if (!std::isfinite(actual) || !std::isfinite(predicted)) continue;

            total_error += std::fabs(actual - predicted);
            valid_sensor_count++;
        }

        if (valid_sensor_count == 0) return 0.0;

        const double valid_sensor_score = clamp01(valid_sensor_count / 2.0);
        const double avg_sensor_error = total_error / valid_sensor_count;
        const double sensor_match_score = clamp01((8.0 - avg_sensor_error) / 8.0);
        const double concentration_score = particle_concentration_score(estimate);
        const double delta_score = chassis_delta_score(estimate, current_pose);

        return clamp01(
            0.40 * sensor_match_score +
            0.25 * concentration_score +
            0.20 * valid_sensor_score +
            0.15 * delta_score
        );
    }

    void update_chassis_pose_from_mcl(const robot_position& estimate) {
        const std::uint32_t curr_time = pros::millis();

        /*
        const auto current = drive::chassis.getPose();
        const double trust = trust_mcl_now(estimate, current);

        if (trust >= MCL_SETPOSE_TRUST_THRESHOLD &&
            curr_time - last_pose_correction_ms >= MCL_SETPOSE_COOLDOWN_MS) {
            drive::chassis.setPose(estimate.x, estimate.y, current.theta);
            last_pose_correction_ms = curr_time;
        }
        */

        if (curr_time - last_estimate_print_ms >= MCL_ESTIMATE_PRINT_INTERVAL_MS) {
            printf("MCL est=(%.3f, %.3f, %.6f)\n", estimate.x, estimate.y, estimate.theta);
            last_estimate_print_ms = curr_time;
        }
    }


    //updates particle weights based on sensor readings
    void update_weights() {
        if (particles.empty()) return;

        int nan_log_sum_count = 0;
        //calculate log weight for each particle
        for (size_t i = 0; i < particles.size(); i++) {
            sensors::range_readings expected = range::raycast(particles[i]);
            double log_sum = 0.0;
            int valid_measurements = 0;
            for (int m = 0; m < 4; m++) {
                const double actual = sensors::distance_readings.direction[m];
                const double predicted = expected.direction[m];

                if (!std::isfinite(actual) || !std::isfinite(predicted)) {
                    continue;
                }

                log_sum += log_gaussian(actual, predicted);
                valid_measurements++;
            }

            if (valid_measurements == 0) {
                log_sum = 0.0;
            }

            if (!std::isfinite(log_sum)) {
                nan_log_sum_count++;
                if (nan_debug_prints < 5) {
                    printf("MCL NaN log_w at particle %zu pos=(%.2f, %.2f, %.4f)\n",
                           i, particles[i].x, particles[i].y, particles[i].theta);
                    printf("  actual  = [%.3f, %.3f, %.3f, %.3f]\n",
                           sensors::distance_readings.front,
                           sensors::distance_readings.left,
                           sensors::distance_readings.back,
                           sensors::distance_readings.right);
                    printf("  expected= [%.3f, %.3f, %.3f, %.3f]\n",
                           expected.front, expected.left, expected.back, expected.right);
                    nan_debug_prints++;
                }
                log_sum = 0.0;
            }

            particles[i].log_w = log_sum;
        }

        //find maximum log weight for numerical stability
        double max_log_w = -INFINITY;
        for (size_t i = 0; i < particles.size(); i++) {
            if (std::isfinite(particles[i].log_w) && particles[i].log_w > max_log_w) {
                max_log_w = particles[i].log_w;
            }
        }

        if (!std::isfinite(max_log_w)) {
            set_uniform_weights();
            return;
        }

        //convert log weights to regular weights and sum
        double sum_w = 0.0;
        for (size_t i = 0; i < particles.size(); i++) {
            if (!std::isfinite(particles[i].log_w)) {
                particles[i].w = 0.0;
                continue;
            }

            particles[i].w = fast_exp(static_cast<float>(particles[i].log_w - max_log_w));
            if (!std::isfinite(particles[i].w)) {
                particles[i].w = 0.0;
            }
            sum_w += particles[i].w;
        }

        if ((!std::isfinite(max_log_w) || !std::isfinite(sum_w)) && nan_debug_prints < 8) {
            printf("MCL invalid weights: max_log_w=%f sum_w=%f nan_log_sum_count=%d\n",
                   max_log_w, sum_w, nan_log_sum_count);
            printf("  current odom=(%.3f, %.3f, %.6f)\n",
                   odom::chassis_odom_position.x,
                   odom::chassis_odom_position.y,
                   odom::chassis_odom_position.theta);
            printf("  current range=[%.3f, %.3f, %.3f, %.3f]\n",
                   sensors::distance_readings.front,
                   sensors::distance_readings.left,
                   sensors::distance_readings.back,
                   sensors::distance_readings.right);
            nan_debug_prints++;
        }

        //handle edge case where all weights are zero
        if (!std::isfinite(sum_w) || sum_w <= 0.0) {
            set_uniform_weights();
            return;
        }

        //normalize weights to sum to 1
        const double inv_sum_w = 1.0 / sum_w;
        for (size_t i = 0; i < particles.size(); i++) {
            particles[i].W = particles[i].w * inv_sum_w;
        }
    }

    //calculates weighted average position from all particles
    robot_position estimate_position () {
        robot_position est = {
            odom::chassis_odom_position.x,
            odom::chassis_odom_position.y,
            odom::chassis_odom_position.theta
        };
        double total_weight = 0.0;

        est.x = 0.0;
        est.y = 0.0;
        for (auto& particle : particles) {
            if (!std::isfinite(particle.W)) continue;
            est.x += particle.x * particle.W;
            est.y += particle.y * particle.W;
            total_weight += particle.W;
        }

        if (std::isfinite(total_weight) && total_weight > 0.0) {
            est.x /= total_weight;
            est.y /= total_weight;
        } else {
            est.x = odom::chassis_odom_position.x;
            est.y = odom::chassis_odom_position.y;
        }

        if ((!std::isfinite(est.x) || !std::isfinite(est.y)) && nan_debug_prints < 10) {
            printf("MCL est_pos became NaN: est=(%f, %f, %f) odom=(%.3f, %.3f, %.6f)\n",
                   est.x, est.y, est.theta,
                   odom::chassis_odom_position.x,
                   odom::chassis_odom_position.y,
                   odom::chassis_odom_position.theta);
            printf("  first particle W/log_w=(%f, %f) pos=(%.2f, %.2f, %.4f)\n",
                   particles.empty() ? NAN : particles[0].W,
                   particles.empty() ? NAN : particles[0].log_w,
                   particles.empty() ? NAN : particles[0].x,
                   particles.empty() ? NAN : particles[0].y,
                   particles.empty() ? NAN : particles[0].theta);
            nan_debug_prints++;
        }
        return est;
    }

    //determines if resampling is needed based on effective sample size
    bool determine_ess () {
        double inv_ess = 0.0;
        for (auto& element : particles) {
            if (!std::isfinite(element.W)) return false;
            inv_ess += element.W * element.W;
        }

        if (!std::isfinite(inv_ess) || inv_ess <= 0.0) return false;

        double ess = 1.0 / inv_ess;
        return ess < N_T;
    }

    //resamples particles based on weights using low variance resampling
    void resample () {
        std::vector<Particle> resampled_particles(N);
        double U_one = rng_gen.uniform() * ONE_N;
        double cumulative_weight = particles[0].W;
        std::size_t i = 0;
        //low variance resampling algorithm
        for (std::size_t j = 1; j <= N; ++j) {
            double U_j = U_one + (static_cast<double>(j - 1) / N);
            while (U_j > cumulative_weight && i + 1 < N) {
                ++i;
                cumulative_weight += particles[i].W;
            }
            resampled_particles[j - 1] = particles[i];
            resampled_particles[j - 1].w = 1.0;
            resampled_particles[j - 1].W = 1.0 / N;
        }

        particles.swap(resampled_particles);
    }

    //propagates particles forward using odometry with added noise
    void state_sample () {
        const double noise_x_stddev = std::sqrt(EPSILON.x_x);
        const double noise_y_stddev = std::sqrt(EPSILON.y_y);
        const double noise_theta_stddev = std::sqrt(EPSILON.theta_theta);
        const double theta = odom::chassis_odom_position.theta;

        //update each particle with odometry delta plus noise
        for (auto& particle : particles) {
            particle.x += delta_odom_pos.x + rng_gen.normal(0.0, SIGMA_X);
            particle.y += delta_odom_pos.y + rng_gen.normal(0.0, SIGMA_Y);
            particle.theta = theta;
            //reset particles that go out of bounds
            if (particle.x < X_MIN || particle.x > X_MAX ||
                particle.y < Y_MIN || particle.y > Y_MAX) {
                particle.x = X_MIN + rng_gen.uniform() * (X_MAX - X_MIN);
                particle.y = Y_MIN + rng_gen.uniform() * (Y_MAX - Y_MIN);
            }
        }
    }

    //initializes MCL system with uniformly distributed particles
    void init_mcl (const lemlib::Pose& init_pose) {
        odom::init_odom(init_pose.x, init_pose.y, init_pose.theta);
        if (!task_is_active(odom_task_handle)) {
            odom_task_handle = pros::Task::create(
                odom::update_odom_position,
                TASK_PRIORITY_DEFAULT + 2,
                TASK_STACK_DEPTH_DEFAULT,
                "Custom Odom"
            );
        }

        drive::chassis.setPose(init_pose);
        mcl_initialized = false;
        if (!range_system_initialized) {
            range::init_range_system();
            range_system_initialized = true;
        }

        sensors::update_range_sensors();
        particles.resize(N);
        prev_odom_pos = odom::chassis_odom_position;
        delta_odom_pos = {0.0, 0.0, 0.0};
        est_pos = {
            odom::chassis_odom_position.x,
            odom::chassis_odom_position.y,
            odom::chassis_odom_position.theta
        };
        nan_debug_prints = 0;
        last_estimate_print_ms = 0;
        printf("MCL init: odom=(%.3f, %.3f, %.6f) particles=%zu\n",
               odom::chassis_odom_position.x,
               odom::chassis_odom_position.y,
               odom::chassis_odom_position.theta,
               particles.size());
        //initialize particles uniformly across field
        for (int i=0; i<N; i++) {
            particles[i].x = X_MIN + rng_gen.uniform() * (X_MAX-X_MIN);
            particles[i].y = Y_MIN + rng_gen.uniform() * (Y_MAX-Y_MIN);
            particles[i].theta = odom::chassis_odom_position.theta;
            particles[i].w = ONE_N;
            particles[i].W = ONE_N;
            particles[i].log_w = 0.0;
        }
        //initial weight update and resampling
        update_weights();
        est_pos = estimate_position();
        printf("MCL init est=(%f, %f, %f)\n", est_pos.x, est_pos.y, est_pos.theta);
        if (determine_ess()) {
            resample();
        }
        mcl_initialized = true;
        if (!task_is_active(mcl_update_task_handle)) {
            mcl_update_task_handle = pros::Task::create(
                mcl::mcl_update,
                TASK_PRIORITY_DEFAULT + 3,
                TASK_STACK_DEPTH_DEFAULT,
                "MCL Update"
            );
        }
    }

    //main MCL update loop running in background task
    void mcl_update () {
        while (true) {
            // Uncomment this block to disable MCL while in driver control / opcontrol.
            /*
            if (!pros::competition::is_autonomous() && !pros::competition::is_disabled()) {
                mcl_initialized = false;
                mcl_update_task_handle = nullptr;
                return;
            }
            */

            if (!mcl_initialized || particles.empty()) {
                pros::delay(5);
                continue;
            }

            //get latest sensor readings
            sensors::update_range_sensors();
            //calculate odometry change since last update
            delta_odom_pos = odom::chassis_odom_position - prev_odom_pos;
            prev_odom_pos = odom::chassis_odom_position;
            //propagate particles with motion model
            state_sample();
            //update particle weights based on sensor readings
            update_weights();
            //update estimated position with mutex protection
            pos_update_mutex.take(TIMEOUT_MAX);
            est_pos = estimate_position();
            update_chassis_pose_from_mcl(est_pos);
            pos_update_mutex.give();
            //resample if effective sample size is too low
            if (determine_ess()) {
                resample();
            }
            pros::delay(5);
        }

    }
}