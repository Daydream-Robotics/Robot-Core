#include "mapping/staticLayer.hpp"

#include <stdexcept>

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