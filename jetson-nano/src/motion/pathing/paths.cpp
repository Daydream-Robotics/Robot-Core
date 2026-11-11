#include "motion/pathing/paths.hpp"
#include "utils/mathHelpers.hpp"
#include <cmath>
#include <cstdio>
#include <fstream>

std::vector<ALS_Path> Path::buildAllPathsFromJerryIO(const std::string& filePath, Pose& outStartPose, std::vector<PathFlag>& outFlags, double sampleSpacing) {
    std::vector<ALS_Path> newPaths;
    outFlags.clear();

    std::vector<double> startHeadings;
    std::vector<std::vector<Waypoint>> rawPaths = parseFile(filePath, startHeadings);

    std::fprintf(stderr, "[PATH] parseFile returned %zu paths\n", rawPaths.size());
    if (rawPaths.empty()) return newPaths;

    // JerryIO headings already use the Pose convention, only deg -> rad
    double initialHeading = std::isnan(startHeadings[0]) ? 0.0 : startHeadings[0];
    double initialHeadingRad = utils::wrapAngle(initialHeading * (M_PI / 180.0));
    std::fprintf(stderr, "[PATH] Initial heading set to %f rad\n", initialHeadingRad);

    // paths stay in the JerryIO field frame, so the pose source must start from this pose
    outStartPose = {rawPaths[0][0].x, rawPaths[0][0].y, initialHeadingRad};

    for (std::size_t pathIndex = 0; pathIndex < rawPaths.size(); pathIndex++) {
        const std::vector<Waypoint>& rawPath = rawPaths[pathIndex];
        ALS_Path tempAlsPath;
        bool success = tempAlsPath.buildFromPoints(rawPath, sampleSpacing);

        if (!success) {
            std::fprintf(stderr, "[PATH] Path %zu build failure, skipped\n", pathIndex);
            continue;
        }

        // the file's heading is where the robot faces, the spline's is the direction of travel.
        // facing more than 90 deg away from the travel direction means the robot backs along the path
        PathFlag flag = PathFlag::FORWARDS;
        if (!std::isnan(startHeadings[pathIndex])) {
            double robotHeading = startHeadings[pathIndex] * (M_PI / 180.0);
            double travelHeading = tempAlsPath.getSamples().front().heading;
            if (std::fabs(utils::wrapAngle(robotHeading - travelHeading)) > M_PI / 2.0) {
                flag = PathFlag::REVERSE;
            }
        }

        newPaths.push_back(tempAlsPath);
        outFlags.push_back(flag);
        std::fprintf(stderr, "[PATH] Path %zu build success (%zu points, %s)\n", pathIndex, rawPath.size(),
                     flag == PathFlag::REVERSE ? "reverse" : "forwards");
    }

    std::fprintf(stderr, "[PATH] buildAllPathsFromJerryIO finished, returning %zu paths\n", newPaths.size());
    return newPaths;
}


std::vector<std::vector<Waypoint>> Path::parseFile(const std::string& filePath, std::vector<double>& outStartHeadings) {
    std::vector<std::vector<Waypoint>> raw_paths;
    std::vector<Waypoint> currentPath;
    outStartHeadings.clear();
    // start heading of the path being read, NaN until its first point
    double currentStartHeading = NAN;

    // open master path file
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::fprintf(stderr, "[PATH] Failed to open file: %s\n", filePath.c_str());
        return raw_paths;
    }

    std::string line;
    bool readingPoints = false;
    int lineCount = 0;

    while (std::getline(file, line)) {
        lineCount++;
        // handle windows CRLF
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }

        // check for new path start
        if (line.find("#PATH-POINTS-START") != std::string::npos) {
            // save previous path if exists
            if (!currentPath.empty()) {
                raw_paths.push_back(currentPath);
                outStartHeadings.push_back(currentStartHeading);
                currentPath.clear();
            }
            currentStartHeading = NAN;
            readingPoints = true;
            continue;
        }

        // end for JSON footer
        if (line.find("#PATH.JERRYIO-DATA") != std::string::npos) {
            break;
        }

        if (!readingPoints || line.empty()) continue;

        double x = 0.0, y = 0.0, v = 0.0, h = 0.0;
        int parsed = std::sscanf(line.c_str(), "%lf,%lf,%lf,%lf", &x, &y, &v, &h);
        if (parsed >= 3) {
            // a path's first point carries the robot heading at its start
            if (currentPath.empty() && parsed >= 4) {
                currentStartHeading = h;
            }

            // cm -> in
            Waypoint newWp = {x / 2.54, y / 2.54, v};

            // Prevent adding duplicate points.
            if (!currentPath.empty()) {
                double dx = newWp.x - currentPath.back().x;
                double dy = newWp.y - currentPath.back().y;
                if (std::hypot(dx, dy) < 1e-5) {
                    std::fprintf(stderr, "[PATH] Skipping duplicate point at line %d\n", lineCount);
                    continue;
                }
            }

            currentPath.push_back(newWp);
        }
    }
    // last path (also when the file has no JSON footer)
    if (!currentPath.empty()) {
        raw_paths.push_back(currentPath);
        outStartHeadings.push_back(currentStartHeading);
    }
    std::fprintf(stderr, "[PATH] File reading complete. Processed %d lines, found %zu paths\n", lineCount, raw_paths.size());

    return raw_paths;
}
