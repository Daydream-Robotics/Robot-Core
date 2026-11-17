#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

// A 2D position in the field coordinate system (inches).
// TODO: Reuse the shared Point type once its definition is confirmed
struct Point {
    double x; 
    double y;
}; 

/**
 * @class StaticLayer
 * @brief Stores fixed field geometry for LiDAR localization. 
 * 
 * Contains sampled points representing field walls and
 * permanent obstacles. The map remains unchanged during 
 * robot operation. 
 */
class StaticLayer {
    public: 
        /**
         * @brief Loads and samples the field geometry 
         * @param fieldFile Path to the YAML field configuration 
         * @param overrideFile Optional field-specific adjustments
         * @param voxelSizeIn Spatial grid cell size in inches
         * @param spacingIn Maximum distance between sampled points
         * @return True if the map loads successfully.
         */
        bool load(
            const std::string& fieldFile, 
            const std::string& overrideFile, 
            double voxelSizeIn, 
            double spacingIn
        ); 

        /**
         * @brief Finds the nearest map point to a query point 
         * @param q Position to search around. 
         * @param maxDist Maximum allowed search distance in inches. 
         * @return Nearest point, or std::nullopt if none is found. 
         */
        std::optional<Point> nearest (
            const Point& q, 
            double maxDist
        ) const; 

        /**
         * @brief Returns all sampled map points. 
         */
        const std::vector<Point>& points() const; 

        /**
         * @brief Returns the source shape for a sampled point 
         */
        std::string_view sourceName(
            size_t pointindex
        ) const; 

    private: 

        // Sampled coordinates of all fixed field surfaces. 
        std::vector<Point> m_points; 

        // Shape name associated with each sampled point
        std::vector<std::string> m_sourceNames; 

        // Spatial grid configuration 
        double m_voxelSizeIn = 1.0; 

        /**
         * @brief Samples evenly spaced points along a lined segment. 
         * @param start Starting coordinate of the segment. 
         * @param end Ending coordinate of the segment.
         * @param spacing Maximum distance between sample points. 
         * @param sourceName Name of the wall or field element. 
         */
        void sampleSegment(
            const Point& start, 
            const Point& end, 
            double spacing, 
            const std::string& sourceName
        );

        /**
         * @brief Samples points around a circular field element. 
         * @param center Center position of the circle
         * @param radius Radius of the circle in inches
         * @param spacing Maximum spacing between samples in inches. 
         * @param sourceName Name of the field element.
         */
        void sampleCircle(
            const Point& center, 
            double radius, 
            double spacing, 
            const std::string& sourceName
        ); 

        /**
         * @brief Samples the perimeter of a closed polygon. 
         * @param vertices Polygon vertices in boundary order. 
         * @param spacing Maximum spacing between samples. 
         * @param sourceName Name of the field element. 
         */
        void samplePolygon(
            const std::vector<Point>& vertices, 
            double spacing, 
            const std::string& sourceName
        );
};