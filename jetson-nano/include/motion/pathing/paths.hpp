#ifndef _PATHS_HPP_
#define _PATHS_HPP_

#include "localization/odometry.hpp"
#include "motion/pathing/arclengthSplining.hpp"
#include <string>
#include <vector>

/**
 * @class Path
 * @brief Parses JerryIO path files and converts them into usable splines
 * @note points stay in JerryIO's field frame (field-centre origin), converted from cm to inches.
 *       JerryIO headings already use the Pose convention (0 along +y, 90 deg along +x)
 */
class Path {
    public:
        Path() = default;

        /**
         * @brief Loads a JerryIO file and builds splined paths
         * @param filePath Path to the JerryIO .txt export
         * @param outStartPose Set to the first point of the first path with the file's starting heading (rad)
         * @param sampleSpacing Distance between spline samples (inches)
         * @returns Vector of all splined paths, in file order. a path that fails to build is skipped
         */
        static std::vector<ALS_Path> buildAllPathsFromJerryIO(const std::string& filePath, Pose& outStartPose, double sampleSpacing = 0.25);

    private:
        /**
         * @brief Parses a jerryIO file and saves the x, y, and v, at each point along the path
         * @param filePath Path to the JerryIO .txt export
         * @param outInitialHeading Reference to store the initial path heading (deg)
         * @returns Vectors of raw waypoints from the file, in inches
         */
        static std::vector<std::vector<Waypoint>> parseFile(const std::string& filePath, double& outInitialHeading);
};

#endif
