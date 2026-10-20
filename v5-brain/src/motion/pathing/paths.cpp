#include "motion/pathing/paths.hpp"
#include "main.h"
#include "utils/helpers.hpp"

std::vector<ALS_Path> paths;
Pose autonStartPose = {0.0, 0.0, 0.0};

std::vector<ALS_Path> Path::buildAllPathsFromJerryIO(const std::string& filePath, double sampleSpacing) {
    std::vector<ALS_Path> newPaths;

    printf("[PATH] buildAllPathsFromJerryIO called for %s\n", filePath.c_str());
    double initialHeading = 0.0;
    printf("[PATH] Calling parseFile\n");
    std::vector<std::vector<Waypoint>> rawPaths = parseFile(filePath, initialHeading);

    printf("[PATH] parseFile returned %zu paths\n", rawPaths.size());
    if (rawPaths.empty()) return newPaths;
    
    // convert jerryio heading to robot heading
    initialHeading = 90.0 - initialHeading;
    double initalHeadingRad = normalizeAngle(initialHeading * (M_PI / 180.0));

    printf("[PATH] Initial heading set to %f rad\n", initalHeadingRad);
    // paths stay in the field frame (JerryIO GPS coords: field center origin), odom is seeded with the start pose
    autonStartPose = {rawPaths[0][0].x, rawPaths[0][0].y, initalHeadingRad};

    int pathIndex = 0;
    for (auto& rawPath : rawPaths) {
        printf("[PATH] Processing path %d, size %zu\n", pathIndex, rawPath.size());
        ALS_Path tempAlsPath;
        printf("[PATH] Building ALS_Path for path %d\n", pathIndex);
        bool sucess = tempAlsPath.buildFromPoints(rawPath, sampleSpacing);

        if (sucess) {
            newPaths.push_back(tempAlsPath);
            printf("[PATH] Path %d build success\n", pathIndex);
            pros::lcd::print(2, "[PATH] Path build success");
        } else {
            printf("[PATH] Path %d build failure\n", pathIndex);
            pros::lcd::print(2, "[PATH] Path build failure");
        }
        pathIndex++;
    }

    printf("[PATH] buildAllPathsFromJerryIO finished, returning %zu paths\n", newPaths.size());
    return newPaths;
}


std::vector<std::vector<Waypoint>> Path::parseFile(const std::string& filePath, double& outInitalHeading) {
    printf("[PATH] parseFile starting for %s\n", filePath.c_str());
    std::vector<std::vector<Waypoint>> raw_paths;
    std::vector<Waypoint> currentPath;

    // open master path file
    std::ifstream file(filePath);
    if (!file.is_open()) {
        printf("[Trajectory] Failed to open file: %s\n", filePath.c_str());
        pros::lcd::print(1, "[Trajectory] Failed to open file: %s", filePath.c_str());
        return raw_paths;
    }
    printf("[PATH] File opened successfully\n");

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
                printf("[PATH] Pushing path to raw_paths (size %zu) at line %d\n", currentPath.size(), lineCount);
                raw_paths.push_back(currentPath);
                currentPath.clear();
            }
            readingPoints = true;
            continue;
        }

        // end for JSON footer
        if (line.find("#PATH.JERRYIO-DATA") != std::string::npos) {
            if (!currentPath.empty()) {
                printf("[PATH] End of paths reached at line %d, pushing final path (size %zu)\n", lineCount, currentPath.size());
                raw_paths.push_back(currentPath);
            }
            break;
        }

        if (!readingPoints || line.empty()) continue;

        double x = 0.0, y = 0.0, v = 0.0, h = 0.0;
        int parsed = std::sscanf(line.c_str(), "%lf,%lf,%lf,%lf", &x, &y, &v, &h);
        if (parsed >= 3) {
            // set init heading
            if (!initialHeadingSet) {
                outInitalHeading = h;
                initialHeadingSet = true;
                printf("[PATH] Found initial heading: %f\n", h);
            }

            Waypoint newWp = {x / 2.54, y / 2.54, v};

            // Prevent adding duplicate points.
            if (!currentPath.empty()) {
                double dx = newWp.x - currentPath.back().x;
                double dy = newWp.y - currentPath.back().y;
                if (std::hypot(dx, dy) < 1e-5) {
                    printf("[PATH] Skipping duplicate point at line %d\n", lineCount);
                    continue;
                }
            }

            // add points converted to inches
            currentPath.push_back(newWp);
        }
    }
    printf("[PATH] File reading complete. Processed %d lines, found %zu paths\n", lineCount, raw_paths.size());
    file.close();

    return raw_paths;
}
