#pragma once
#include <stdio.h>
#include <vector>
#include "odometry.hpp"

struct Sample {
    double t = 0.0; // where in spline param
    double s = 0.0; // distance from start
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0; // direction of motion
    double curvature = 0.0;
};

struct SplineSegment {
    //cubic polynomial on [t0, t1]
    // f(t) = a + bu + cu^2 + du^3, where u = t - t0
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    double d = 0.0;

    double t0 = 0.0;
    double t1 = 0.0;
};


class CubicSpline {
    public:
        CubicSpline() = default;

        // build a cubic spline through points (t[i], values[i])
        bool buildSpline(const std::vector<double>& t, const std::vector<double>& values);

        // Eval spline & derivs
        double evaluate(double tQuery) const;
        double evalFirstDeriv(double tQuery) const;
        double evalSecondDeriv(double tQuery) const;

        // Utilities

        bool isValid() const;
        std::size_t segmentCount() const;
        const std::vector<SplineSegment>& getSegments() const;
    
    private:
        std::vector<SplineSegment> m_segments;

        std::size_t findSegmentIndex(double tQuery) const;

        bool m_valid = false;
};

class ALS_Path {
    public:
        ALS_Path() = default;

        // ===============================================
        //    MAIN PATH BUILDING & QUERY INTERFACE
        //  Primary External Query Interface for lookahead point
        // ===============================================
        Position returnLookaheadPoint(const Position& currentPos, double lookaheadDistance);
        
        // Main build function, computes paramterization, fit, and sample table
        bool buildFromPoints(const std::vector<Position>& points, double sampleSpacing = 0.25);

        // Closest Point helper
        std::size_t findClosestSampleIndex(const Position& robotPos, std::size_t startIdx = 0, std::size_t endIdx = static_cast<std::size_t>(-1)) const;
        
        // Utilities
        double getMaxAbsCurvatureInRange(double sStart, double sEnd) const;
        
        const std::vector<double>& getParameters() const;
        const std::vector<Sample>& getSamples() const;
        const CubicSpline& getXSpline() const;
        const CubicSpline& getYSpline() const;
        
        bool isValid() const;
        double getTotalLength() const;
        
    private:
        // Build Helpers
        static std::vector<double> computeChordLengthParameters(const std::vector<Position>& points);
        
        void buildSamples(double sampleSpacing);
        double arcLengthToParameter(double sQuery) const;
    
        CubicSpline m_splineX;
        CubicSpline m_splineY;
        
        // Original params for points
        std::vector<double> m_parameters;
        
        // Dense lookup able for arc-length queries
        std::vector<Sample> m_samples;
        
        // Query points and geometry
        Position getPointAtParameter(double tQuery) const;
        Position getPointAtArcLength(double sQuery) const;
        
        double getHeadingAtParameter(double tQuery) const;
        double getCurvatureAtParameter(double tQuery) const;
        
        
        
        // globals
        double m_totalLength = 0.0;
        std::size_t m_lastIndex = 0; // for efficient closest point queries
        bool m_valid = false;
};

