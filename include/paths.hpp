#pragma once
#include "odometry.hpp"
#include "arclengthSplining.hpp"
#include <vector>
#include <fstream>

#ifndef PATHS_HPP
#define PATHS_HPP 3.14159265358979323846

enum PathName {
    FIRST_PATH,
    SECOND_PATH,
    THIRD_PATH,
    COUNT
};

class Path {
    public:
        Path() = default;
        static std::vector<ALS_Path> buildAllPathsFromJerryIO(const std::string& filePath, double sampleSpacing = 0.25);

    private:
        /**
         * @brief Parses a jerryIO file and saves the x, y, and v, at each point along the path
         * 
         */
        static std::vector<std::vector<Waypoint>> parseFile(const std::string& filePath, double& outInitalHeading);
        
        
        static std::vector<Waypoint> normalizePath(const std::vector<Waypoint>& rawPath, Pose initialPose);
};

extern std::vector<ALS_Path> paths;

#endif