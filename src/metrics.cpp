#include "metrics.hpp"
#include "helpers.hpp"
#include <cmath>
#include <cstdio>

// -------------------------------------------------------
// PathFollowingTracker
// -------------------------------------------------------

void PathFollowingTracking::update(const Pose& pose, const Sample& nearest) {

    double dx = std::cos(nearest.heading);
    double dy = std::sin(nearest.heading);
    double ex = pose.x - nearest.x;
    double ey = pose.y - nearest.y;
    double cte = dx * ey - dy * ex; 

    maxCTE = std::max(maxCTE, std::abs(cte));
    finalCTE = cte;

    n++;
    double delta = cte - avgCTE;
    avgCTE  += delta / n;
    rmsCTE += delta * (cte - avgCTE);

    int sign = (cte >= 0.0) ? 1 : -1;
    if (lastSign != 0 && sign != lastSign) signChanges++;
    lastSign = sign;
}

double PathFollowingTracking::getRMSCTE() const {
    return n > 0 ? std::sqrt(rmsCTE / n) : 0.0;
}

void PathFollowingTracking::recordPositionError(const Pose& pose, const Sample& target) {
    double dx = pose.x - target.x;
    double dy = pose.y - target.y;
    posErrorAtPoint = std::sqrt(dx*dx + dy*dy);
}

void PathFollowingTracking::reset() {
    maxCTE = avgCTE = rmsCTE = finalCTE = posErrorAtPoint = 0.0;
    n = signChanges = lastSign = 0;
}

// -------------------------------------------------------
// HeadingTracker
// -------------------------------------------------------

void HeadingAccuracyTracking::update(const Pose& pose, const Sample& nearest, double timestamp_s) {
    // nearest.heading is in radians (direction of motion along path)
    double pathHeadingDeg = nearest.heading * 180.0 / M_PI;
    double robotHeadingDeg = pose.theta * 180.0 / M_PI;
    double error = angleDiffDeg(robotHeadingDeg, pathHeadingDeg);

    maxError = std::max(maxError, std::abs(error));

    // Oscillation: heading rate sign flip
    if (n > 1) {
        double rate = error - lastError;
        int sign = (rate >= 0) ? 1 : -1;
        if (lastRateSign != 0 && sign != lastRateSign) oscillations++;
        lastRateSign = sign;

        // Heading settling time
        double rateAbs = std::abs(error - lastError);
        if (rateAbs < SETTLE_RATE_THRESHOLD) {
            if (!inSettleWindow) {
                inSettleWindow  = true;
                settleStartTime = timestamp_s;
                settleTickCount = 0;
            }
            settleTickCount++;
            if (settleTickCount >= SETTLE_TICKS_REQUIRED && settlingTime < 0.0) {
                settlingTime = settleStartTime;
            }
        } else {
            inSettleWindow  = false;
            settleTickCount = 0;
            settlingTime    = -1.0;
        }
    }

    lastError = error;

    n++;
    double delta = error - avgError;
    avgError += delta / n;
    rmsError += delta * (error - avgError);
}

double HeadingAccuracyTracking::getRMSError() const {
    return n > 0 ? std::sqrt(rmsError / n) : 0.0;
}

void HeadingAccuracyTracking::reset() {
    maxError = avgError = rmsError = lastError = 0.0;
    n = oscillations = lastRateSign = 0;
    settlingTime = settleStartTime = -1.0;
    inSettleWindow = false;
    settleTickCount = 0;
}

// -------------------------------------------------------
// VelocityTracker
// -------------------------------------------------------

void VelocityTracking::update(const Pose&, const Sample&, double parallelVel) {
    double v = std::abs(parallelVel);
    maxVel = std::max(maxVel, v);
    n++;
    avgVel += (v - avgVel) / n;
}

void VelocityTracking::reset() {
    avgVel = maxVel = 0.0;
    n = 0;
}

// -------------------------------------------------------
// SmoothnessTracker
// -------------------------------------------------------

void SmoothnessTracking::update(double parallelVel, double poseTheta_rad, double timestamp_s) {
    if (!initialized) {
        lastVel   = parallelVel;
        lastTheta = poseTheta_rad;
        lastTime  = timestamp_s;
        initialized = true;
        return;
    }

    double dt = timestamp_s - lastTime;
    if (dt <= 0.0) return;

    // Linear jerk
    double accel = (parallelVel - lastVel) / dt;

    if (hasAccel) {
        double jerk = (accel - lastAccel) / dt;
        peakJerk = std::max(peakJerk, std::abs(jerk));
        n++;
        avgJerk += (std::abs(jerk) - avgJerk) / n;
    }

    lastVel   = parallelVel;
    lastAccel = accel;
    hasAccel  = true;

    // Steering jerk
    double omega = (poseTheta_rad - lastTheta) / dt;

    if (hasOmega) {
        double steerAccel = (omega - lastOmega) / dt;

        if (hasSteerAccel) {
            double steerJerk = (steerAccel - lastSteerAccel) / dt;
            peakSteeringJerk = std::max(peakSteeringJerk, std::abs(steerJerk));
            avgSteeringJerk += (std::abs(steerJerk) - avgSteeringJerk) / n;
        }

        lastSteerAccel = steerAccel;
        hasSteerAccel  = true;
    }

    lastTheta = poseTheta_rad;
    lastOmega = omega;
    hasOmega  = true;

    lastTime = timestamp_s;
}

void SmoothnessTracking::reset() {
    avgJerk = peakJerk = lastVel = lastAccel = 0.0;
    lastTime = -1.0;
    n = 0;
    initialized = false;
    hasAccel    = false;
    lastTheta = lastOmega = lastSteerAccel = 0.0;
    peakSteeringJerk = avgSteeringJerk = 0.0;
    hasOmega     = false;
    hasSteerAccel = false;
}

// -------------------------------------------------------
// TimingTracker
// -------------------------------------------------------

void TimingTracking::start(double timestamp_s) {
    startTime = timestamp_s;
    lastTimestamp = timestamp_s;
    started = true;
}

void TimingTracking::update(double cte, double headingError, double timestamp_s,
                           double cteThreshold, double headingThreshold) {
    if (!started) return;
    double dt = timestamp_s - lastTimestamp;
    if (std::abs(cte) > cteThreshold || std::abs(headingError) > headingThreshold) {
        timeAboveThreshold += dt;
    }
    lastTimestamp = timestamp_s;
}

void TimingTracking::finish(double timestamp_s) {
    if (started) totalTime = timestamp_s - startTime;
}

void TimingTracking::reset() {
    startTime = totalTime = timeAboveThreshold = lastTimestamp = 0.0;
    started = false;
}

// -------------------------------------------------------
// EndpointTracker
// -------------------------------------------------------

void EndpointPerformanceTracking::record(const Pose& finalPose, const Sample& goalSample) {
    double dx = finalPose.x - goalSample.x;
    double dy = finalPose.y - goalSample.y;
    finalPosError = std::sqrt(dx*dx + dy*dy);

    double robotDeg = finalPose.theta * 180.0 / M_PI;
    double goalDeg  = goalSample.heading * 180.0 / M_PI;
    finalHeadError = angleDiffDeg(robotDeg, goalDeg);
}

void EndpointPerformanceTracking::updateSettling(const Pose& pose, const Sample& goal, double timestamp_s) {
    double dx   = pose.x - goal.x;
    double dy   = pose.y - goal.y;
    double dist = std::sqrt(dx*dx + dy*dy);

    double robotDeg = pose.theta * 180.0 / M_PI;
    double goalDeg  = goal.heading * 180.0 / M_PI;
    double headErr  = std::abs(angleDiffDeg(robotDeg, goalDeg));

    distOvershoot    = std::max(distOvershoot, dist);
    headingOvershoot = std::max(headingOvershoot, headErr);

    bool isSettled = (dist < POS_SETTLE_THRESH && headErr < HEADING_SETTLE_THRESH);

    if (!settled && isSettled) {
        timeToSettle = timestamp_s;
        settled = true;
    } else if (settled && !isSettled) {
        correctionCount++;
        settled      = false;
        timeToSettle = -1.0;
    }
}

void EndpointPerformanceTracking::reset() {
    finalPosError = finalHeadError = 0.0;
    timeToSettle    = -1.0;
    correctionCount = 0;
    settleStartTime = -1.0;
    settled         = false;
    distOvershoot   = headingOvershoot = 0.0;
}

// -------------------------------------------------------
// MetricsSystem
// -------------------------------------------------------

void MetricsSystem::startRun(double timestamp_s) {
    path.reset();
    heading.reset();
    velocity.reset();
    smoothness.reset();
    timing.reset();
    endpoint.reset();
    timing.start(timestamp_s);
}

void MetricsSystem::update(const Pose& pose, const Sample& nearest,
                           double parallelVel, double timestamp_s) {
    path.update(pose, nearest);
    heading.update(pose, nearest, timestamp_s);
    velocity.update(pose, nearest, parallelVel);
    smoothness.update(parallelVel, pose.theta, timestamp_s);
    timing.update(path.getFinalCTE(), heading.getFinalError(), timestamp_s);
}

void MetricsSystem::finishRun(const Pose& finalPose, const Sample& goalSample,
                               double timestamp_s) {
    timing.finish(timestamp_s);
    endpoint.record(finalPose, goalSample);
}
