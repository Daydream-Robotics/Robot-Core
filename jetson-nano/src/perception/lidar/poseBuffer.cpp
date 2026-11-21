#include "perception/lidar/poseBuffer.hpp"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>

#include "config/constants.h"
#include "utils/mathHelpers.hpp"

void PoseBuffer::push(uint64_t tUs, const Pose& pose) {
    // one writer, readers wait
    std::lock_guard<std::mutex> lock(m_mutex);

    // drop a non-finite pose
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta)) {
        return;
    }

    // order and gap checks need a previous sample
    if (m_count > 0) {
        // newest stored sample
        const int latest = (m_head - 1 + LIDAR_POSE_BUFFER_CAPACITY) % LIDAR_POSE_BUFFER_CAPACITY;
        const Sample& latestSample = m_samples[latest];

        // drop an out-of-order or duplicate time
        if (tUs <= latestSample.tUs) {
            return;
        }

        // odometry stalled, log it and still store the sample
        if ((tUs - latestSample.tUs) > LIDAR_POSE_GAP_US) {
            std::fprintf(stderr, "[PoseBuffer] odom gap of %" PRIu64 " us\n", tUs - latestSample.tUs);
        }
    }

    // write at the head, the oldest sample is overwritten once full
    m_samples[m_head] = Sample{tUs, pose};
    // move the head forward, wrapping to 0 at the end
    m_head = (m_head + 1) % LIDAR_POSE_BUFFER_CAPACITY;
    // one more sample, up to LIDAR_POSE_BUFFER_CAPACITY
    m_count = std::min(m_count + 1, LIDAR_POSE_BUFFER_CAPACITY);

    // average odom rate over the full buffer
    if (m_count == LIDAR_POSE_BUFFER_CAPACITY) {
        const int oldest = m_head;
        const uint64_t spanUs = tUs - m_samples[oldest].tUs;
        const double hz = (LIDAR_POSE_BUFFER_CAPACITY - 1) * 1e6 / static_cast<double>(spanUs);

        // log once when the rate drops, again only after it recovers
        if (hz < LIDAR_POSE_MIN_HZ && !m_lowRateLogged) {
            std::fprintf(stderr, "[PoseBuffer] odom rate %.1f Hz below %.1f Hz\n", hz, LIDAR_POSE_MIN_HZ);
            m_lowRateLogged = true;
        } 
        else if (hz >= LIDAR_POSE_MIN_HZ) {
            m_lowRateLogged = false;
        }
    }
}

std::optional<Pose> PoseBuffer::poseAt(uint64_t tUs) const {
    std::lock_guard<std::mutex> lock(m_mutex);

    // nothing stored yet
    if (m_count == 0) {
        return std::nullopt;
    }

    // oldest and newest samples
    const int latestIdx = (m_head - 1 + LIDAR_POSE_BUFFER_CAPACITY) % LIDAR_POSE_BUFFER_CAPACITY;
    const int oldestIdx = (m_head - m_count + LIDAR_POSE_BUFFER_CAPACITY) % LIDAR_POSE_BUFFER_CAPACITY;
    const Sample& latestSample = m_samples[latestIdx];
    const Sample& oldestSample = m_samples[oldestIdx];

    // older than the buffer
    if (tUs < oldestSample.tUs) {
        return std::nullopt;
    }

    // newer than the newest sample
    if (tUs >= latestSample.tUs) {
        if (tUs - latestSample.tUs <= LIDAR_POSE_EXTRAPOLATE_US) {
            return latestSample.pose;
        }
        return std::nullopt;
    }

    // binary search over logical positions
    int lo = 0;
    int hi = m_count - 1;
    // keeps lo at or before t and hi after t
    while (hi - lo > 1) {
        const int mid = lo + (hi - lo) / 2;
        const Sample& midSample = m_samples[(oldestIdx + mid) % LIDAR_POSE_BUFFER_CAPACITY];
        if (midSample.tUs <= tUs) {
            lo = mid;
        } 
        else {
            hi = mid;
        }
    }

    // the two samples around t
    const Sample& a = m_samples[(oldestIdx + lo) % LIDAR_POSE_BUFFER_CAPACITY];
    const Sample& b = m_samples[(oldestIdx + hi) % LIDAR_POSE_BUFFER_CAPACITY];

    // t lands on a sample
    if (a.tUs == tUs) {
        return a.pose;
    }

    // odometry stalled between a and b, don't trust the blend
    if (b.tUs - a.tUs > LIDAR_POSE_GAP_US) {
        return std::nullopt;
    }

    // fraction of the way from a to b
    const double alpha = static_cast<double>(tUs - a.tUs) / static_cast<double>(b.tUs - a.tUs);
    return interpolate(a, b, alpha);
}

void PoseBuffer::clear() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_head = 0;
    m_count = 0;
    m_lowRateLogged = false;
}

Pose PoseBuffer::interpolate(const Sample& a, const Sample& b, double alpha) {
    // x and y blended linearly
    const double x = a.pose.x + alpha * (b.pose.x - a.pose.x);
    const double y = a.pose.y + alpha * (b.pose.y - a.pose.y);

    // heading takes the short way around
    const double dTheta = utils::normalizeAngle(b.pose.theta - a.pose.theta);
    const double theta = utils::normalizeAngle(a.pose.theta + alpha * dTheta);

    return Pose{x, y, theta};
}
