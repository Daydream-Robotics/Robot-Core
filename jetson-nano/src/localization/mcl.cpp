//Inclusions
#include "daydream/utils/helpers.hpp"
#include "daydream/motion/odometry.hpp"
#include "daydream/mcl/rangeSensing.hpp"
#include "daydream/mcl/mcl.hpp"
#include "daydream/utils/rng.hpp"

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
    //random number generator, reseeded in initMcl() (millis() is ~0 during static init, so every boot would match)
    rng::Xoshiro128Plus rngGen(1);

    //range sensing system used for raycasting expected sensor readings
    RangeSensing rangeSensing;

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
    RobotPosition estPos = {0,0,0};
    int nanDebugPrints = 0;
    bool rangeSystemInitialized = false;
    bool mclInitialized = false;
    pros::task_t mclUpdateTaskHandle = nullptr;
    // std::uint32_t lastPoseCorrectionMs = 0;
    std::uint32_t lastEstimatePrintMs = 0;
    //previous odometry position for delta calculation
    RobotPosition prevOdomPos;
    //change in odometry position since last update
    RobotPosition deltaOdomPos;

    //motion noise stddev per update, scaled with how far odom moved (inches of noise per inch moved)
    //TODO: tune
    constexpr double MOTION_NOISE_PER_INCH = 0.1;
    //small stddev added every update so particles keep some spread while the robot is still
    constexpr double MOTION_NOISE_FLOOR = 0.01;

    //vector storing all particles
    std::vector<Particle> particles;

    //mutex for thread-safe position updates
    pros::Mutex posUpdateMutex;

    //wraps angle to range [-pi, pi]
    inline double wrapToPi(double a) {
        if (a > M_PI)  a -= 2.0 * M_PI;
        if (a <= -M_PI) a += 2.0 * M_PI;
        return a;
    }

    //subtracts two odometry positions with angle wrapping
    inline RobotPosition operator-(const RobotPosition& a,
        const RobotPosition& b) {
        return {
            a.x - b.x,
            a.y - b.y,
            wrapToPi(a.theta - b.theta)
        };
    }

    //calculates log of gaussian probability
    inline double logGaussian(double z, double mu) {
        double r = z - mu;
        return -(r * r) * INV_2_SIGMA2 - 0.5 * LOG_NORM;
    }

    //clamps value to 0 or 1
    inline double clamp01(double value) {
        if (value < 0.0) return 0.0;
        if (value > 1.0) return 1.0;
        return value;
    }

    //measures how close mcl estimate to current chassis pose
    double chassisDeltaScore(const RobotPosition& estimate, const RobotPosition& currentPose) {
        const double dx = estimate.x - currentPose.x;
        const double dy = estimate.y - currentPose.y;
        //calcs position delta
        const double positionDelta = std::sqrt(dx * dx + dy * dy);

        //clamos the value from 0 to 1, where 1 means the psoition matches and 0.0 means they are at least 18 in apart, values between decrease linearly
        return clamp01((18.0 - positionDelta) / 18.0);
    }

    //checks whether a PROS task handle represents a currently usable task
    bool taskIsActive(pros::task_t taskHandle) {
        if (taskHandle == nullptr) return false;

        const auto state = pros::c::task_get_state(taskHandle);
        return state != pros::E_TASK_STATE_DELETED && state != pros::E_TASK_STATE_INVALID;
    }

    //fast approximation of exponential function, for x <= 0
    inline float fastExp(float x) {
        //below -256 the base goes negative and the even power makes it huge, e^-256 is 0 anyway
        if (x <= -256.0f) return 0.0f;
        x = 1.0f + x / 256.0f;
        x *= x; x *= x; x *= x; x *= x;
        x *= x; x *= x; x *= x; x *= x;
        return x;
    }

    //initialisez weights for all particles
    void setUniformWeights() {
        if (particles.empty()) return;

        const double inverseN = 1.0 / particles.size();
        for (auto& element : particles) {
            element.weight = 1.0;
            element.normalizedWeight = inverseN;
            element.logWeight = 0.0;
        }
    }

    //scores how tightly weighted particles cluster around the MCL estimate.
    double particleConcentrationScore(const RobotPosition& estimate) {
        if (particles.empty()) return 0.0;

        double totalWeight = 0.0;
        double weightedSqRadius = 0.0;

        for (const auto& particle : particles) {
            if (!std::isfinite(particle.normalizedWeight)) continue;

            const double dx = particle.x - estimate.x;
            const double dy = particle.y - estimate.y;

            // accumulate each particle's weighted squared distance from the estimate.
            weightedSqRadius += particle.normalizedWeight * (dx * dx + dy * dy);
            totalWeight += particle.normalizedWeight;
        }

        //a nonpositive total weight cannot produce a meaningful concentration score.
        if (!std::isfinite(totalWeight) || totalWeight <= 0.0) return 0.0;

        //convert the weighted squared spread into an RMS radius before scoring it.
        const double rmsRadius = std::sqrt(weightedSqRadius / totalWeight);
        return clamp01((18.0 - rmsRadius) / 18.0);
    }

    double trustMclNow(const RobotPosition& estimate, const RobotPosition& currentPose) {
        //do not trust an estimate until MCL is initialized and its pose is valid.
        if (!mclInitialized || particles.empty()) return 0.0;
        if (!std::isfinite(estimate.x) || !std::isfinite(estimate.y) || !std::isfinite(estimate.theta)) {
            return 0.0;
        }

        Particle estimateParticle = {
            estimate.x,
            estimate.y,
            estimate.theta,
            1.0,
            1.0,
            0.0
        };

        //predict sensor readings at the MCL estimate for comparison with the real sensors.
        const RangeReadings expected = rangeSensing.raycast(estimateParticle);
        double totalError = 0.0;
        int validSensorCount = 0;

        for (int i = 0; i < 4; i++) {
            const double actual = distanceReadings.direction(i);
            const double predicted = expected.direction(i);

            //ignore a sensor direction when either reading is invalid.
            if (!std::isfinite(actual) || !std::isfinite(predicted)) continue;

            totalError += std::fabs(actual - predicted);
            validSensorCount++;
        }

        if (validSensorCount == 0) return 0.0;

        //convert sensor coverage and average error into normalized confidence scores.
        const double validSensorScore = clamp01(validSensorCount / 2.0);
        const double avgSensorError = totalError / validSensorCount;
        const double sensorMatchScore = clamp01((8.0 - avgSensorError) / 8.0);
        const double concentrationScore = particleConcentrationScore(estimate);
        const double deltaScore = chassisDeltaScore(estimate, currentPose);

        //combine independent confidence signals into one score from 0 to 1.
        return clamp01(
            0.40 * sensorMatchScore +
            0.25 * concentrationScore +
            0.20 * validSensorScore +
            0.15 * deltaScore
        );
    }

    void updateChassisPoseFromMcl(const RobotPosition& estimate) {
        const std::uint32_t currTime = pros::millis();

        /*
        // apply an MCL correction only after the estimate passes the trust threshold.
        const Pose current = odom.getPose();
        const RobotPosition currentPosition = {current.x, current.y, current.theta};
        const double trust = trustMclNow(estimate, currentPosition);

        if (trust >= MCL_SETPOSE_TRUST_THRESHOLD &&
            currTime - lastPoseCorrectionMs >= MCL_SETPOSE_COOLDOWN_MS) {
            odom.setPose({estimate.x, estimate.y, current.theta});
            lastPoseCorrectionMs = currTime;
        }
        */

        //limit console output so the background MCL task does not print every update.
        if (currTime - lastEstimatePrintMs >= MCL_ESTIMATE_PRINT_INTERVAL_MS) {
            printf("MCL est=(%.3f, %.3f, %.6f)\n", estimate.x, estimate.y, estimate.theta);
            lastEstimatePrintMs = currTime;
        }
    }


    //updates particle weights based on sensor readings
    void updateWeights() {
        if (particles.empty()) return;

        int nanLogSumCount = 0;
        //calculate log weight for each particle
        for (size_t i = 0; i < particles.size(); i++) {
            RangeReadings expected = rangeSensing.raycast(particles[i]);
            double logSum = 0.0;
            int validMeasurements = 0;
            for (int m = 0; m < 4; m++) {
                const double actual = distanceReadings.direction(m);
                const double predicted = expected.direction(m);

                if (!std::isfinite(actual) || !std::isfinite(predicted)) {
                    continue;
                }

                logSum += logGaussian(actual, predicted);
                validMeasurements++;
            }

            //no sensor could be compared, so there's no evidence for this particle. Give it zero weight
            //(0 would be the best possible log weight). If every particle ends up here, the weights go uniform
            if (validMeasurements == 0) {
                particles[i].logWeight = -INFINITY;
                continue;
            }

            if (!std::isfinite(logSum)) {
                nanLogSumCount++;
                if (nanDebugPrints < 5) {
                    printf("MCL NaN log_w at particle %zu pos=(%.2f, %.2f, %.4f)\n",
                           i, particles[i].x, particles[i].y, particles[i].theta);
                    printf("  actual  = [%.3f, %.3f, %.3f, %.3f]\n",
                           distanceReadings.front,
                           distanceReadings.left,
                           distanceReadings.back,
                           distanceReadings.right);
                    printf("  expected= [%.3f, %.3f, %.3f, %.3f]\n",
                           expected.front, expected.left, expected.back, expected.right);
                    nanDebugPrints++;
                }
                logSum = 0.0;
            }

            particles[i].logWeight = logSum;
        }

        //find maximum log weight for numerical stability
        double maxLogWeight = -INFINITY;
        for (size_t i = 0; i < particles.size(); i++) {
            if (std::isfinite(particles[i].logWeight) && particles[i].logWeight > maxLogWeight) {
                maxLogWeight = particles[i].logWeight;
            }
        }

        if (!std::isfinite(maxLogWeight)) {
            setUniformWeights();
            return;
        }

        //convert log weights to regular weights and sum
        double sumWeight = 0.0;
        for (size_t i = 0; i < particles.size(); i++) {
            if (!std::isfinite(particles[i].logWeight)) {
                particles[i].weight = 0.0;
                continue;
            }

            particles[i].weight = fastExp(static_cast<float>(particles[i].logWeight - maxLogWeight));
            if (!std::isfinite(particles[i].weight)) {
                particles[i].weight = 0.0;
            }
            sumWeight += particles[i].weight;
        }

        if ((!std::isfinite(maxLogWeight) || !std::isfinite(sumWeight)) && nanDebugPrints < 8) {
            printf("MCL invalid weights: max_log_w=%f sum_w=%f nan_log_sum_count=%d\n",
                   maxLogWeight, sumWeight, nanLogSumCount);
                 const Pose currentPose = odom.getPose();
                 printf("  current odom=(%.3f, %.3f, %.6f)\n",
                     currentPose.x,
                     currentPose.y,
                     currentPose.theta);
            printf("  current range=[%.3f, %.3f, %.3f, %.3f]\n",
                   distanceReadings.front,
                   distanceReadings.left,
                   distanceReadings.back,
                   distanceReadings.right);
            nanDebugPrints++;
        }

        //handle edge case where all weights are zero
        if (!std::isfinite(sumWeight) || sumWeight <= 0.0) {
            setUniformWeights();
            return;
        }

        //normalize weights to sum to 1
        const double invSumWeight = 1.0 / sumWeight;
        for (size_t i = 0; i < particles.size(); i++) {
            particles[i].normalizedWeight = particles[i].weight * invSumWeight;
        }
    }

    //calculates weighted average position from all particles
    RobotPosition estimatePosition() {
        const Pose currentPose = odom.getPose();
        RobotPosition est = {
            currentPose.x,
            currentPose.y,
            currentPose.theta
        };
        double totalWeight = 0.0;

        est.x = 0.0;
        est.y = 0.0;
        for (auto& particle : particles) {
            if (!std::isfinite(particle.normalizedWeight)) continue;
            est.x += particle.x * particle.normalizedWeight;
            est.y += particle.y * particle.normalizedWeight;
            totalWeight += particle.normalizedWeight;
        }

        if (std::isfinite(totalWeight) && totalWeight > 0.0) {
            est.x /= totalWeight;
            est.y /= totalWeight;
        } else {
            est.x = currentPose.x;
            est.y = currentPose.y;
        }

        if ((!std::isfinite(est.x) || !std::isfinite(est.y)) && nanDebugPrints < 10) {
            printf("MCL est_pos became NaN: est=(%f, %f, %f) odom=(%.3f, %.3f, %.6f)\n",
                   est.x, est.y, est.theta,
                   currentPose.x,
                   currentPose.y,
                   currentPose.theta);
            printf("  first particle W/log_w=(%f, %f) pos=(%.2f, %.2f, %.4f)\n",
                   particles.empty() ? NAN : particles[0].normalizedWeight,
                   particles.empty() ? NAN : particles[0].logWeight,
                   particles.empty() ? NAN : particles[0].x,
                   particles.empty() ? NAN : particles[0].y,
                   particles.empty() ? NAN : particles[0].theta);
            nanDebugPrints++;
        }
        return est;
    }

    //determines if resampling is needed based on effective sample size
    bool determineEss() {
        double invEss = 0.0;
        for (auto& element : particles) {
            if (!std::isfinite(element.normalizedWeight)) return false;
            invEss += element.normalizedWeight * element.normalizedWeight;
        }

        if (!std::isfinite(invEss) || invEss <= 0.0) return false;

        double ess = 1.0 / invEss;
        return ess < N_T;
    }

    //resamples particles based on weights using low variance resampling
    void resample() {
        std::vector<Particle> resampledParticles(N);
        double uOne = rngGen.uniform() * ONE_N;
        double cumulativeWeight = particles[0].normalizedWeight;
        std::size_t i = 0;
        //low variance resampling algorithm
        for (std::size_t j = 1; j <= N; ++j) {
            double uJ = uOne + (static_cast<double>(j - 1) / N);
            while (uJ > cumulativeWeight && i + 1 < N) {
                ++i;
                cumulativeWeight += particles[i].normalizedWeight;
            }
            resampledParticles[j - 1] = particles[i];
            resampledParticles[j - 1].weight = 1.0;
            resampledParticles[j - 1].normalizedWeight = 1.0 / N;
        }

        particles.swap(resampledParticles);
    }

    //propagates particles forward using odometry with added noise
    void stateSample() {
        const double theta = odom.getPose().theta;

        //noise grows with distance moved, so a still robot doesn't spread its particles out
        const double distMoved = std::hypot(deltaOdomPos.x, deltaOdomPos.y);
        const double noiseStddev = MOTION_NOISE_FLOOR + MOTION_NOISE_PER_INCH * distMoved;

        //update each particle with odometry delta plus noise
        for (auto& particle : particles) {
            particle.x += deltaOdomPos.x + rngGen.normal(0.0, noiseStddev);
            particle.y += deltaOdomPos.y + rngGen.normal(0.0, noiseStddev);
            particle.theta = theta;
            //reset particles that go out of bounds
            if (particle.x < X_MIN || particle.x > X_MAX ||
                particle.y < Y_MIN || particle.y > Y_MAX) {
                particle.x = X_MIN + rngGen.uniform() * (X_MAX - X_MIN);
                particle.y = Y_MIN + rngGen.uniform() * (Y_MAX - Y_MIN);
            }
        }
    }

    //initializes MCL system with uniformly distributed particles
    void initMcl(const Pose& initPose) {
        odom.setPose(initPose);
        //no-op if initialize() already started it
        odom.startTask();

        //seed from the time MCL starts and the battery voltage, both vary between runs
        const std::uint32_t seed = static_cast<std::uint32_t>(pros::micros()) ^
            (static_cast<std::uint32_t>(pros::battery::get_voltage()) << 16);
        rngGen = rng::Xoshiro128Plus(seed);

        mclInitialized = false;
        if (!rangeSystemInitialized) {
            rangeSensing.initRangeSystem();
            rangeSystemInitialized = true;
        }

        updateRangeSensors();
        particles.resize(N);
        prevOdomPos = {initPose.x, initPose.y, initPose.theta};
        deltaOdomPos = {0.0, 0.0, 0.0};
        estPos = {
            initPose.x,
            initPose.y,
            initPose.theta
        };
        nanDebugPrints = 0;
        lastEstimatePrintMs = 0;
        printf("MCL init: odom=(%.3f, %.3f, %.6f) particles=%zu\n",
             initPose.x,
             initPose.y,
             initPose.theta,
               particles.size());
        //initialize particles uniformly across field
        for (int i=0; i<N; i++) {
            particles[i].x = X_MIN + rngGen.uniform() * (X_MAX-X_MIN);
            particles[i].y = Y_MIN + rngGen.uniform() * (Y_MAX-Y_MIN);
            particles[i].theta = initPose.theta;
            particles[i].weight = ONE_N;
            particles[i].normalizedWeight = ONE_N;
            particles[i].logWeight = 0.0;
        }
        //initial weight update and resampling
        updateWeights();
        estPos = estimatePosition();
        printf("MCL init est=(%f, %f, %f)\n", estPos.x, estPos.y, estPos.theta);
        if (determineEss()) {
            resample();
        }
        mclInitialized = true;
        if (!taskIsActive(mclUpdateTaskHandle)) {
            mclUpdateTaskHandle = pros::Task::create(
                mcl::mclUpdate,
                //below the odom task (DEFAULT + 1) so a long MCL update can't delay odometry
                TASK_PRIORITY_DEFAULT,
                TASK_STACK_DEPTH_DEFAULT,
                "MCL Update"
            );
        }
    }

    //main MCL update loop running in background task
    void mclUpdate() {
        while (true) {
            // Uncomment this block to disable MCL while in driver control / opcontrol.
            /*
            if (!pros::competition::is_autonomous() && !pros::competition::is_disabled()) {
                mclInitialized = false;
                mclUpdateTaskHandle = nullptr;
                return;
            }
            */

            if (!mclInitialized || particles.empty()) {
                pros::delay(5);
                continue;
            }

            //get latest sensor readings
            updateRangeSensors();
            //calculate odometry change since last update
            const Pose currentPose = odom.getPose();
            deltaOdomPos = {
                currentPose.x - prevOdomPos.x,
                currentPose.y - prevOdomPos.y,
                wrapToPi(currentPose.theta - prevOdomPos.theta)
            };
            prevOdomPos = {currentPose.x, currentPose.y, currentPose.theta};
            //propagate particles with motion model
            stateSample();
            //update particle weights based on sensor readings
            updateWeights();
            //update estimated position with mutex protection
            posUpdateMutex.take(TIMEOUT_MAX);
            estPos = estimatePosition();
            updateChassisPoseFromMcl(estPos);
            posUpdateMutex.give();
            //resample if effective sample size is too low
            if (determineEss()) {
                resample();
            }
            pros::delay(5);
        }

    }
}