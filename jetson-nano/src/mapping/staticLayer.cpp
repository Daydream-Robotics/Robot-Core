#include "mapping/staticLayer.hpp"

#include <stdexcept>
#include <cmath>
#include <algorithm>

/** 
 * @brief Returns all sampled points in the 
 * static field map. 
 */
const std::vector<Point>& StaticLayer::points() const {
    return m_points; 
}

/**
 * @brief Returns the name of the field element that 
 * generated a point. 
 */
std::string_view StaticLayer::sourceName(size_t pointIndex)
const {
    if (pointIndex >= m_sourceNames.size()) {
        throw std::out_of_range("Invalid static map point index"); 
    } 
    return m_sourceNames[pointIndex]; 
}

/**
 * @brief Samples evenly spaced points along a line segment. 
 */
void StaticLayer::sampleSegment(
    const Point& start, 
    const Point& end, 
    double spacing, 
    const std::string& sourceName
) {
    // Calculate the difference between the endpoints
    const double dx = end.x - start.x; 
    const double dy = end.y - start.y; 

    // Calculate the total length of the segment
    const double length = std::hypot(dx, dy);

    // Ensure spacing is valid
    if (!std::isfinite(spacing) || spacing <= 0.0) {
        throw std::invalid_argument(
            "Sampling spacing must be positive and finite"
        ); 
    }

    // Determine the number of intervals
    const size_t intervals = std::max(
        static_cast<size_t>(1), 
        static_cast<size_t>(std::ceil(length / spacing))
    ); 

    // Generate points along the segment
    for (size_t i = 0; i <= intervals; ++i) {

        // Interpolation factor between 0 and 1
        const double t =
            static_cast<double>(i) /
            static_cast<double>(intervals); 

        Point point{
            start.x + t * dx, 
            start.y + t * dy
        }; 

        // Store the point and its source
        m_points.push_back(point); 
        m_sourceNames.push_back(sourceName); 
    }
}

/** 
 * @brief Samples points along a circle's circumference. 
 */
void StaticLayer::sampleCircle(
    const Point& center, 
    double radius, 
    double spacing, 
    const std::string& sourceName
) {
    // Validate the radius and spacing
    if (!std::isfinite(radius) || radius <= 0.0 ||
        !std::isfinite(spacing) || spacing <= 0.0) {
        throw std::invalid_argument(
            "Circle radius and spacing must be positive and finite"
        ); 
    }

    if (!std::isfinite(center.x) || !std::isfinite(center.y)) {
        throw std::invalid_argument(
            "Circle center coordinates must be finite"
        ); 
    }

    // Calculate the circumference: 2 * pi *r
    const double circumference = 2.0 * std::acos(-1.0) * radius; 

    // Find the number of points needed
    const size_t count = std::max(
        static_cast<size_t>(3), 
        static_cast<size_t>(std::ceil(circumference / spacing))
    ); 

    // Generate points around the circumference
    for (size_t i = 0; i < count; ++i) {

        const double angle =
            2.0 * std::acos(-1.0) *
            static_cast<double>(i) /
            static_cast<double>(count); 

        Point point{
            center.x + radius * std::cos(angle), 
            center.y + radius * std::sin(angle)
        };

        m_points.push_back(point); 
        m_sourceNames.push_back(sourceName); 
    }
}

/**
 * @brief Samples every edge of a closed polygon
 */
void StaticLayer::samplePolygon(
    const std::vector<Point>& vertices, 
    double spacing, 
    const std::string& sourceName
) {
    // A polygon requires at least 3 vertices
    if (vertices.size() < 3) {
        throw std::invalid_argument(
            "A polygon requires at least three vertices"
        ); 
    }

    // Generate points along each polygon edge
    for (size_t i = 0; i < vertices.size(); ++i) {

        const Point& start = vertices[i]; 

        // Wrap back to the first vertex at the last edge 
        const Point& end =
            vertices[(i+1) % vertices.size()]; 

        sampleSegment(start, end, spacing, sourceName); 
    }
}

/**
 * @brief Converts a field coordinate into a grid cell. 
 */
StaticLayer::CellKey StaticLayer::cellForPoint(
    const Point& point
) const {
    return {
        static_cast<int64_t>(
            std::floor(point.x / m_voxelSizeIn)
        ), 
        static_cast<int64_t>(
            std::floor(point.y / m_voxelSizeIn)
        )
    }; 
}

/**
 * @brief Builds the spatial hash grid from sampled points. 
 */
void StaticLayer::buildGrid() {

    // Clear any previously stored grid data
    m_grid.clear(); 

    // Insert each sampled point into its corresponding cell
    for (const Point& point : m_points) {

        const CellKey cell = cellForPoint(point); 

        m_grid[cell].push_back(point); 
    }
}