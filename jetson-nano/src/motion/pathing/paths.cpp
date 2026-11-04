#include "motion/pathing/paths.hpp"
#include "utils/mathHelpers.hpp"
#include <cmath>
#include <cstdio>
#include <fstream>

std::vector<ALS_Path> Path::buildAllPathsFromJerryIO(const std::string& filePath, Pose& outStartPose, double sampleSpacing) {
    std::vector<ALS_Path> newPaths;

    double initialHeading = 0.0;
    std::vector<std::vector<Waypoint>> rawPaths = parseFile(filePath, initialHeading);

    std::fprintf(stderr, "[PATH] parseFile returned %zu paths\n", rawPaths.size());
    if (rawPaths.empty()) return newPaths;

    // JerryIO headings already use the Pose convention, only deg -> rad
    double initialHeadingRad = utils::wrapAngle(initialHeading * (M_PI / 180.0));
    std::fprintf(stderr, "[PATH] Initial heading set to %f rad\n", initialHeadingRad);

    // paths stay in the JerryIO field frame, so the pose source must start from this pose
    outStartPose = {rawPaths[0][0].x, rawPaths[0][0].y, initialHeadingRad};

    int pathIndex = 0;
    for (const auto& rawPath : rawPaths) {
        ALS_Path tempAlsPath;
        bool success = tempAlsPath.buildFromPoints(rawPath, sampleSpacing);

        if (success) {
            newPaths.push_back(tempAlsPath);
            std::fprintf(stderr, "[PATH] Path %d build success (%zu points)\n", pathIndex, rawPath.size());
        } else {
            std::fprintf(stderr, "[PATH] Path %d build failure, skipped\n", pathIndex);
        }
        pathIndex++;
    }

    std::fprintf(stderr, "[PATH] buildAllPathsFromJerryIO finished, returning %zu paths\n", newPaths.size());
    return newPaths;
}


std::vector<std::vector<Waypoint>> Path::parseFile(const std::string& filePath, double& outInitialHeading) {
    std::vector<std::vector<Waypoint>> raw_paths;
    std::vector<Waypoint> currentPath;

    // open master path file
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::fprintf(stderr, "[PATH] Failed to open file: %s\n", filePath.c_str());
        return raw_paths;
    }

    std::string line;
    bool readingPoints = false;

    bool initialHeadingSet = false;
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
                currentPath.clear();
            }
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
            // set init heading (only the first point of the file carries one)
            if (!initialHeadingSet && parsed >= 4) {
                outInitialHeading = h;
                initialHeadingSet = true;
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
    }
    std::fprintf(stderr, "[PATH] File reading complete. Processed %d lines, found %zu paths\n", lineCount, raw_paths.size());

    return raw_paths;
}
