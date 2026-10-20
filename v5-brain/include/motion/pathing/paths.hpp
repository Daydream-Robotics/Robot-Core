#pragma once
#include "localization/odometry.hpp"
#include "motion/pathing/arclengthSplining.hpp"
#include <vector>
#include <fstream>

#ifndef PATHS_HPP
#define PATHS_HPP 3.14159265358979323846

/**
 * @brief Index identifiers to map named pathways
 * @note number of paths must match number of paths in jerry io file
 */
enum PathName {
    FIRST_PATH,
    SECOND_PATH,
    THIRD_PATH,
    FOURTH_PATH,
    COUNT
};

/**
 * @class Path
 * @brief Parses path text files and converts them into usable splines 
 */
class Path {
    public:
        Path() = default;

        /**
         * @brief Loads JerryIO file and builds splined paths
         * @param filePath Path to the JerryIO file on the SD card
         * @param sampleSpacing Distance between spline samples (inches)
         * @returns Vector of all splined paths
         */
        static std::vector<ALS_Path> buildAllPathsFromJerryIO(const std::string& filePath, double sampleSpacing = 0.25);

    private:
        /**
         * @brief Parses a jerryIO file and saves the x, y, and v, at each point along the path
         * @param filePath Path to the JerryIO file on the SD card
         * @param outInitialHeading Reference to store the initial path heading
         * @returns Vectors of raw waypoints from the file
         */
        static std::vector<std::vector<Waypoint>> parseFile(const std::string& filePath, double& outInitalHeading);
        
};

/**
 * @brief Global vector holding all auton paths
 */
extern std::vector<ALS_Path> paths;

/**
 * @brief Field-frame pose at the start of the first path (field center origin, inches/rads)
 * @note set by buildAllPathsFromJerryIO, odom is seeded with it at the start of auton
 */
extern Pose autonStartPose;

#endif
