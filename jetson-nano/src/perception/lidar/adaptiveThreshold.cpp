#include "perception/lidar/adaptiveThreshold.hpp"

#include "utils/mathHelpers.hpp"

#include <cmath>

AdaptiveThreshold::AdaptiveThreshold(double tau0In, double deltaMinIn, double rMaxIn)
    : m_tau0(tau0In), m_deltaMin(deltaMinIn), m_rMax(rMaxIn) {
    reset();
}

void AdaptiveThreshold::reset() {
    double sigma0 = m_tau0 / 3.0;
    m_sumSq = sigma0 * sigma0;
    m_count = 1;
}

double AdaptiveThreshold::sigma() const {
    return std::sqrt(m_sumSq / m_count);
}

double AdaptiveThreshold::tau() const {
    return 3.0 * sigma();
}

void AdaptiveThreshold::update(const Pose& tInit, const Pose& tResult) {
    // how far ICP moved the position from the guess (in)
    double deltaTrans = std::hypot(tResult.x - tInit.x, tResult.y-tInit.y);

    // how far ICP turned the heading from the guess (rad), wrapped
    double deltaTheta = utils::wrapAngle(tResult.theta-tInit.theta);

    // how far that heading change moves a point at max range (chord length, in)
    // a small heading error moves far points the most
    double deltaRot = 2.0 * m_rMax * std::sin(std::abs(deltaTheta)/2.0);

    // worst-case point displacement caused by the guess error
    double delta = deltaRot + deltaTrans;

    // only count real corrections
    if (delta > m_deltaMin) {
        // add the squared displacement to the running sum that sigma() averages
        m_sumSq += delta * delta;
        m_count++;
    }
}