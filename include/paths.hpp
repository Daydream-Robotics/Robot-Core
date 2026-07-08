#pragma once
#include "odometry.hpp"
#include "arclengthSplining.hpp"
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
        
        /**
         * @brief Transforms the raw path coordinates relative to the robot's starting posisition and orientation
         * @param rawPath Vector of raw waypoints from the file relative to the field
         * @param initalPose The starting pose of the robot relative to the field
         * @returns Vector of transformed waypoints
         */
        static std::vector<Waypoint> normalizePath(const std::vector<Waypoint>& rawPath, Pose initialPose);
};

/**
 * @brief Global vector holding all auton paths
 */
extern std::vector<ALS_Path> paths;

#endif