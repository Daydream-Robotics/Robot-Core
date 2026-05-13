#include "trajectory.hpp"
#include "main.h"
#include "helpers.hpp"

#include <fstream>


void Trajectory::addNode(TrajectoryNode node) {
    m_nodes.push_back(node);
}


double Trajectory::getTotalTime() const {
    if (m_nodes.empty()) return 0.0;
    return m_nodes.back().time;
}


TrajectoryNode Trajectory::sample(double curTime) const {
    if (m_nodes.empty()) return {};

    if (curTime <= m_nodes.front().time) return m_nodes.front();
    if (curTime >= m_nodes.back().time) return m_nodes.back();

    // find node at this time
    int low = 0;
    int high = m_nodes.size() - 1;
    while (low <= high) {
        int mid = low + (high - low) / 2;
        if (m_nodes[mid].time < curTime) {
            low = mid + 1;
        } else if (m_nodes[mid].time > curTime) {
            high = mid - 1;
        } else {
            return m_nodes[mid];
        }
    }

    // interpolate the location between the next and previous point
    const TrajectoryNode& prev = m_nodes[low - 1];
    const TrajectoryNode& next = m_nodes[low];

    double tDiff = next.time - prev.time;
    if (tDiff == 0.0) return next;

    double tFrac = (curTime - prev.time) / tDiff;

    TrajectoryNode result;
    result.time = curTime;
    result.pose.x = prev.pose.x + tFrac * (next.pose.x - prev.pose.x);
    result.pose.y = prev.pose.y + tFrac * (next.pose.y - prev.pose.y);

    double deltaTheta = next.pose.theta - prev.pose.theta;
    while (deltaTheta > std::numbers::pi) deltaTheta -= 2.0 * std::numbers::pi;
    while (deltaTheta < -std::numbers::pi) deltaTheta += 2.0 * std::numbers::pi;
    result.pose.theta = normalizeAngle(prev.pose.theta + tFrac * deltaTheta);

    result.velocity = prev.velocity + tFrac * (next.velocity - prev.velocity);
    result.omega = prev.omega + tFrac * (next.omega - prev.omega);

    return result;
}


std::vector<Trajectory> Trajectory::loadAllFromJerryIO(const std::string& filePath) {
    std::vector<Trajectory> trajectories;

    double initialHeading = 0.0;
    std::vector<std::vector<Waypoint>> rawPaths = parseFile(filePath, initialHeading);

    if (rawPaths.empty()) return trajectories;

    // setup inital point reference frame
    double initalX = rawPaths[0][0].x;
    double initalY = rawPaths[0][0].y;
    
    // convert jerryio heading to robot heading
    initialHeading = 90.0 - initialHeading;
    double initalHeadingRad = normalizeAngle(initialHeading * (M_PI / 180.0));

    Pose initialPose = {rawPaths[0][0].x, rawPaths[0][0].y, initalHeadingRad};

    for (auto& rawPath : rawPaths) {
        if (rawPath.size() < 2) continue;

        std::vector<Position> geomPoints;
        for (auto& waypoint : rawPath) {
            geomPoints.push_back({waypoint.x, waypoint.y});
        }

        ALS_Path tempAlsPath;
        tempAlsPath.buildFromPoints(geomPoints, 0.25);
        std::vector<Sample> splines = tempAlsPath.getSamples();

        normalizePath(splines, initialPose);
        
        Trajectory newTrajectory = profileTrajectory(splines, rawPath);
        trajectories.push_back(newTrajectory);
    }

    return trajectories;
}


std::vector<std::vector<Waypoint>> Trajectory::parseFile(const std::string& filePath, double& outInitalHeading) {
    std::vector<std::vector<Waypoint>> raw_paths;
    std::vector<Waypoint> currentPath;

    // open master path file
    std::ifstream file(filePath);
    if (!file.is_open()) {
        pros::lcd::print(1, "[Trajectory] Failed to open file: %s", filePath.c_str());
        return raw_paths;
    }

    std::string line;
    bool readingPoints = false;

    bool initialHeadingSet = false;

    while (std::getline(file, line)) {
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
            if (!currentPath.empty()) {
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
            }

            // add points converted to inches
            currentPath.push_back({x / 2.54, y / 2.54, v});
        }
    }
    file.close();

    return raw_paths;
}


void Trajectory::normalizePath(std::vector<Sample>& splinedData, Pose initialPose) {
    for (Sample& sample : splinedData) {
        double dx = sample.x - initialPose.x;
        double dy = sample.y - initialPose.y;

        sample.x = dx * std::cos(initialPose.theta) + dy * std::sin(initialPose.theta);
        sample.y = -dx * std::sin(initialPose.theta) + dy * std::cos(initialPose.theta);

        sample.heading = normalizeAngle(sample.heading - initialPose.theta);
    }
}


Trajectory Trajectory::profileTrajectory(const std::vector<Sample>& splinedData, const std::vector<Waypoint>& rawPath) {
    Trajectory newTrajectory;

    double currentTime = 0.0;
    double rpmToInchesPerSec = (std::numbers::pi * DRIVE_WHEEL_DIAMETER_INCHES) / 60.0;

    for (int i = 0; i < splinedData.size(); ++i) {
        const Sample& sample = splinedData[i];

        // find nearest point and set velocity
        double target_v = 0.0;
        double minDist = std::numeric_limits<double>::infinity();
        for (const auto& wp : rawPath) {
            double dist = std::hypot(wp.x - sample.x, wp.y - sample.y);
            if (dist < minDist) {
                minDist = dist;
                target_v = wp.v * rpmToInchesPerSec;
            }
        }

        // calculate angular velocity
        double target_w = target_v * sample.curvature;

        // calc dt = ds / v
        if (i > 0) {
            double ds = sample.s - splinedData[i - 1].s;
            double prev_v = newTrajectory.m_nodes.back().velocity;
            double avgVel = (target_v + prev_v) / 2.0;
            
            if (std::abs(avgVel) > 0.001) {
                currentTime += ds / std::abs(avgVel);;
            } else {
                currentTime += 0.001;
            }
        }

        newTrajectory.addNode({
            currentTime,
            {sample.x, sample.y, sample.heading},
            target_v,
            target_w
        });

    return newTrajectory;
}