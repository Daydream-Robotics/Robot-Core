#include "arclengthSplining.hpp"
#include <cmath>
#include <cstdio>
#include <algorithm>
#include "sd_card_logging.hpp" 

// ----------------------------------------
//             MAIN INTERFACE
// ----------------------------------------



Position ALS_Path::returnLookaheadPoint(const Position& curPosition) {
    if (m_samples.empty()) {
        LOG("No samples in returnLookaheadPoint");
        return Position{};
    }

    // Get current closest point on line
    std::size_t closestIdx = findClosestSampleIndex(curPosition, m_lastIndex, getSamples().size());
    const Sample& currentSample = getSamples()[closestIdx];

    m_lastIndex = closestIdx; // update last index for faster search

    // ================================
    // LOOKAHEAD CALCULATIONS
    // equation = some constant + curvature gain * abs(curvature at closest point), clamped between min & max
    // base = some constant
    // min = minimum clamp
    // max = maximum clamp
    // curvature = curvature at CURRENT CLOSEST POINT
    // ================================
    double baseLookahead = 5.0;
    double minLookahead  = 2.0;
    double maxLookahead  = 12.0;
    double curvatureGain = 20.0;

    double lookaheadDistance = baseLookahead / (1.0 + curvatureGain * std::abs(currentSample.curvature));
    lookaheadDistance = std::clamp(lookaheadDistance, minLookahead, maxLookahead);


    double targetS = currentSample.s + lookaheadDistance; 
    if (targetS > m_totalLength) {
        targetS = m_totalLength;
    }

    return getPointAtArcLength(targetS);
}


// ----------------------------------------
//    CUBICSPLINING & FUNCTION EVAL
// ----------------------------------------
bool CubicSpline::buildSpline(const std::vector<double>& t, const std::vector<double>& values) {
    m_segments.clear();
    m_valid = false;

    const std::size_t n = t.size();
    if (n < 2) {
        LOG("Invalid T length");
        return false;
    } else if (values.size() != n) {
        LOG("T length does not match Path length");
        return false;
    }

    // Ensure strictly increasing t
    for (std::size_t i = 1; i < n; i++) {
        if (t[i] <= t[i - 1]) {
            return false;
        }
    }

    // SPECIAL CASE: ONLY 2 POINTS; 
    if (n == 2) {
        double h = t[1] - t[0];
        double slope = (values[1] - values[0]) / h;

        SplineSegment seg;
        seg.a = values[0];
        seg.b = slope;
        seg.c = 0.0;
        seg.d = 0.0;
        seg.t0 = t[0];
        seg.t1 = t[1];

        m_segments.push_back(seg);
        m_valid = true;
        return true;
    }

    // h[i] = t[i+1] - t[i]
    std::vector<double> h(n-1);
    for (std::size_t i = 0; i < n - 1; i++) {
        h[i] = t[i+1] - t[i];
    }

    // Solve for c coefficients using tridiagonal linear system (look it up)
    std::vector<double> alpha(n, 0.0);
    for (std::size_t i = 1; i < n-1; i++) {
        alpha[i] = ((3.0 / h[i]) * (values[i+1] - values[i])) - ((3.0 / h[i-1]) * (values[i] - values[i-1]));
    }

    std::vector<double> l(n, 0.0);
    std::vector<double> mu(n, 0.0);
    std::vector<double> z(n, 0.0);
    
    l[0] = 1.0;
    mu[0] = z[0] = 0.0;


    // Compute l
    for (std::size_t i = 1; i < n - 1; i++) {
        l[i] = 2.0 * (t[i + 1] - t[i - 1]) - h[i - 1] * mu[i - 1];
        if (std::abs(l[i]) < 1e-12) {
            m_segments.clear();
            return false;
        }

        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    l[n - 1] = 1.0;
    z[n - 1] = 0.0;

    std::vector<double> a(n - 1, 0.0);
    std::vector<double> b(n - 1, 0.0);
    std::vector<double> c(n, 0.0);
    std::vector<double> d(n - 1, 0.0);

    c[n - 1] = 0.0;

    for (std::size_t j = n - 1; j-- > 0; ) {
        c[j] = z[j] - mu[j] * c[j + 1];
        b[j] = (values[j + 1] - values[j]) / h[j]
             - h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0;
        d[j] = (c[j + 1] - c[j]) / (3.0 * h[j]);
        a[j] = values[j];
    }

    m_segments.reserve(n - 1);
    for (std::size_t i = 0; i < n - 1; i++) {
        SplineSegment seg;
        seg.a = a[i];
        seg.b = b[i];
        seg.c = c[i];
        seg.d = d[i];
        seg.t0 = t[i];
        seg.t1 = t[i + 1];
        m_segments.push_back(seg);
    }

    m_valid = true;
    return true;
}

// Decides which cubic segment contains the t query
std::size_t CubicSpline::findSegmentIndex(double tQuery) const {
    if (m_segments.empty()) {
        return 0;
    }

    // Clamp to first
    if (tQuery <= m_segments.front().t0) {
        return 0;
    }

    // Clamp on last
    if (tQuery >= m_segments.back().t1) {
        return m_segments.size() - 1;
    }

    std::size_t left = 0;
    std::size_t right = m_segments.size() - 1;

    // binary search
    while (left <= right) {
        std::size_t mid = left + (right - left) / 2;
        const SplineSegment& seg = m_segments[mid];

        if (tQuery < seg.t0) {
            if (mid == 0) {
                return 0;
            }
            right = mid - 1;
        } else if (tQuery > seg.t1) {
            left = mid + 1;
        } else {
            return mid;
        }
    }

    return m_segments.size() - 1;
}

// FUNCTION EVALUATION

double CubicSpline::evaluate(double tQuery) const {
    
    if (!m_valid || m_segments.empty()) {
        LOG("Spline invalid in evaluate");
        return 0.0;
    }
    
    const SplineSegment& seg = m_segments[findSegmentIndex(tQuery)];
    double u = tQuery - seg.t0;

    return seg.a + seg.b * u + seg.c * u*u + seg.d * u*u*u;
}

double CubicSpline::evalFirstDeriv(double tQuery) const {
    if (!m_valid || m_segments.empty()) {
        LOG("Spline invalid in evalFirstDeriv");
        return 0.0;
    }

    const SplineSegment& seg = m_segments[findSegmentIndex(tQuery)];
    double u = tQuery - seg.t0;

    return seg.b + 2.0 * seg.c * u + 3.0 * seg.d * u*u;
}

double CubicSpline::evalSecondDeriv(double tQuery) const {
    if (!m_valid || m_segments.empty()) {
        LOG("Spline invalid in evalSecondDeriv");
        return 0.0;
    }

    const SplineSegment& seg = m_segments[findSegmentIndex(tQuery)];
    double u = tQuery - seg.t0;

    return 2.0 * seg.c + 6.0 * seg.d *u;
}

// UTILITIES

bool CubicSpline::isValid() const {
    return m_valid;
}

std::size_t CubicSpline::segmentCount() const {
    return m_segments.size();
}

const std::vector<SplineSegment>& CubicSpline::getSegments() const {
    return m_segments;
}



// ----------------------------------------
//    ARC-LENGTH PARAM. & PATHBUILDING
// ----------------------------------------
std::vector<double> ALS_Path::computeChordLengthParameters(const std::vector<Position>& points) {
    std::vector<double> t;

    if (points.empty()) {
        LOG("Points empty");
        return t;
    }

    t.resize(points.size());
    t[0] = 0.0;

    for (std::size_t i = 1; i < points.size(); i++) {
        double dx = points[i].x - points[i-1].x;
        double dy = points[i].y - points[i-1].y;
        double dist = std::hypot(dx, dy);
        t[i] = t[i-1] + dist;
    }

    return t;
}

bool ALS_Path::buildFromPoints(const std::vector<Position>& points, double sampleSpacing) {
    m_valid = false;
    m_parameters.clear();
    m_samples.clear();
    m_totalLength = 0.0;
    m_lastIndex = 0;
    
    if (points.size() < 2) {
        LOG("Need at least 2 points to build path");
        return false;
    }

    if (sampleSpacing <= 0.0) {
        LOG("Sample spacing must be positive");
        return false;
    }

    // Compute t[i] from chord lengths
    m_parameters = computeChordLengthParameters(points);

    // Reject duplicates that cause repeated t values (cleanup just in case)
    for (std::size_t i = 1; i < m_parameters.size(); i++) {
        if (m_parameters[i] <= m_parameters[i - 1]) {
            LOG("Points contain duplicate or non-increasing positions");
            return false;
        }
    }

    // Split points into x and y
    std::vector<double> xVals(points.size());
    std::vector<double> yVals(points.size());

    for (std::size_t i = 0; i < points.size(); i++) {
        xVals[i] = points[i].x;
        yVals[i] = points[i].y;
    }

    // Build parametric splines
    if (!m_splineX.buildSpline(m_parameters, xVals)) {
        LOG("Failed to build X spline");
        return false;
    }

    if (!m_splineY.buildSpline(m_parameters, yVals)) {
        LOG("Failed to build Y spline");
        return false;
    }

    // Build sample table for arc-length quieries
    buildSamples(sampleSpacing);

    if (m_samples.empty()) {
        LOG("Failed to build sample table");
        return false;
    }

    m_valid = true;
    return true;
}

Position ALS_Path::getPointAtParameter(double tQuery) const {
    Position p;
    p.x = m_splineX.evaluate(tQuery);
    p.y = m_splineY.evaluate(tQuery);
    return p;
}

Position ALS_Path::getPointAtArcLength(double sQuery) const {
    double tQuery = arcLengthToParameter(sQuery);
    Position p;


    p.x = m_splineX.evaluate(tQuery);
    p.y = m_splineY.evaluate(tQuery);
    return p;
}

double ALS_Path::getHeadingAtParameter(double tQuery) const {
    double dx = m_splineX.evalFirstDeriv(tQuery);
    double dy = m_splineY.evalFirstDeriv(tQuery);
    return std::atan2(dy, dx);
}

// κ(t)=(x′(t)2+y′(t)2)3/2x′(t)y′′(t)−y′(t)x′′(t)​
double ALS_Path::getCurvatureAtParameter(double tQuery) const {
    double dx = m_splineX.evalFirstDeriv(tQuery);
    double dy = m_splineY.evalFirstDeriv(tQuery);
    double ddx = m_splineX.evalSecondDeriv(tQuery);
    double ddy = m_splineY.evalSecondDeriv(tQuery);

    double denom = std::pow(dx * dx + dy * dy, 1.5);
    if (denom < 1e-12) {
        return 0.0;
    }

    return (dx * ddy - dy * ddx) / denom;
}


// BUILD SAMPLE TABLE FOR ARC-LENGTH QUERIES

void ALS_Path::buildSamples(double sampleSpacing) {
    m_samples.clear();
    m_totalLength = 0.0;

    if (!m_splineX.isValid() || !m_splineY.isValid()) {
        LOG("Splines invalid in buildSamples");
        return;
    }

    if (m_parameters.size() < 2) {
        LOG("Not enough parameters to build samples");
        return;
    }

    if (sampleSpacing <= 0.0) {
        LOG("Sample spacing must be positive");
        return;
    }

    const double tStart = m_parameters.front();
    const double tEnd   = m_parameters.back();

    Position prev = getPointAtParameter(tStart);

    Sample first;
    first.t = tStart;
    first.s = 0.0;
    first.x = prev.x;
    first.y = prev.y;
    first.heading = getHeadingAtParameter(tStart);
    first.curvature = getCurvatureAtParameter(tStart);
    m_samples.push_back(first);

    double accumulatedS = 0.0;
    double currentT = tStart;

    while (currentT < tEnd) {
        double dxdt = m_splineX.evalFirstDeriv(currentT);
        double dydt = m_splineY.evalFirstDeriv(currentT);

        double speed = std::hypot(dxdt, dydt);

        double dt;
        if (speed < 1e-9) {
            dt = 1e-3;
        } else {
            dt = sampleSpacing / speed;
        }

        double nextT = currentT + dt;
        if (nextT > tEnd) {
            nextT = tEnd;
        }

        Position curr = getPointAtParameter(nextT);

        double ds = std::hypot(curr.x - prev.x, curr.y - prev.y);
        accumulatedS += ds;

        Sample sample;
        sample.t = nextT;
        sample.s = accumulatedS;
        sample.x = curr.x;
        sample.y = curr.y;
        sample.heading = getHeadingAtParameter(nextT);
        sample.curvature = getCurvatureAtParameter(nextT);

        m_samples.push_back(sample);

        prev = curr;
        currentT = nextT;
    }

    m_totalLength = accumulatedS;
}

std::size_t ALS_Path::findClosestSampleIndex(const Position& robotPos, std::size_t startIdx, std::size_t endIdx) const {
    if (m_samples.empty()) {
        return 0;
    }

    if (startIdx >= m_samples.size()) {
        startIdx = m_samples.size() - 1;
    }

    if (endIdx == static_cast<std::size_t>(-1) || endIdx > m_samples.size()) {
        endIdx = m_samples.size();
    }

    std::size_t closestIdx = startIdx;
    double minDist = std::hypot(robotPos.x - m_samples[startIdx].x, robotPos.y - m_samples[startIdx].y);

    std::size_t count = 0;
    const std::size_t maxCount = m_samples.size() / 5; // worst case, search through 20% of samples before giving up (prevents long search if something goes wrong)

    for (std::size_t i = startIdx + 1; i < endIdx; i++) {
        double dist = std::hypot(robotPos.x - m_samples[i].x, robotPos.y - m_samples[i].y);

        if (dist < minDist) {
            minDist = dist;
            closestIdx = i;
            count = 0;
        } else {
            count++;
        }

        if (count >= maxCount) {
            break;
        }
    }

    return closestIdx;
}

double ALS_Path::arcLengthToParameter(double sQuery) const {
    if (m_samples.empty()) {
        LOG("No samples in arcLengthToParameter");
        return 0.0;
    }

    if (sQuery <= 0.0) {
        return m_samples.front().t;
    }

    if (sQuery >= m_samples.back().s) {
        return m_samples.back().t;
    }

    // Binary search to find the right interval
    std::size_t left = 0;
    std::size_t right = m_samples.size() - 1;
    while (left <= right) {
        std::size_t mid = left + (right - left) / 2;
        const Sample& seg = m_samples[mid];

        if (sQuery < seg.s) {
            if (mid == 0) {
                return m_samples.front().t;
            }
            right = mid - 1;
        } else if (sQuery > seg.s) {
            left = mid + 1;
        } else {
            return m_samples[mid].t;
        }
    }

    // Interpolate between samples[right] and samples[left]
    std::size_t upper = left;
    std::size_t lower = upper - 1;

    const Sample& a = m_samples[lower];
    const Sample& b = m_samples[upper];

    double ds = b.s - a.s;
    if (std::abs(ds) < 1e-12) {
        return a.t;
    }

    double alpha = (sQuery - a.s) / ds;
    return a.t + alpha * (b.t - a.t); //returns a t that is linearly interpolated between a.t and b.t based on where sQuery falls between a.s and b.s
}


// UTILITIES

bool ALS_Path::isValid() const {
    return m_valid;
}

double ALS_Path::getTotalLength() const {
    return m_totalLength;
}

const std::vector<double>& ALS_Path::getParameters() const {
    return m_parameters;
}

const std::vector<Sample>& ALS_Path::getSamples() const {
    return m_samples;
}

const CubicSpline& ALS_Path::getXSpline() const {
    return m_splineX;
}

const CubicSpline& ALS_Path::getYSpline() const {
    return m_splineY;
}