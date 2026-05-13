#pragma once
#include "odometry.hpp"
#include "arclengthSplining.hpp"
#include <vector>

enum TrajectoryName {
    FIRST_PATH,
    SECOND_PATH,
    COUNT
};

struct TrajectoryNode {
    double time;
    
    /**
     * @brief Target X, Y, and Heading (radians)
     */
    Pose pose;

    /**
     * @brief Target linear velocity
     */
    double velocity;

    /**
     * @brief Target angular velocity
     */
    double omega;
};

struct Waypoint {
    double x;
    double y;
    double v;
};

/**
 * @class Trajectory
 */
class Trajectory {
    public:
        Trajectory() = default;

        void addNode(TrajectoryNode node);
        double getTotalTime() const;
        TrajectoryNode sample(double curTime) const;

        /**
         * @brief Parses JerryIO file and builds all contained trajectories
         */
        static std::vector<Trajectory> loadAllFromJerryIO(const std::string& filePath);

    private:
        std::vector<TrajectoryNode> m_nodes;

        static std::vector<std::vector<Waypoint>> parseFile(const std::string& filePath, double& outInitalHeading);
        static void normalizePath(std::vector<Sample>& splinedData, Pose initialPose);
        static Trajectory profileTrajectory(const std::vector<Sample>& splinedData, const std::vector<Waypoint>& rawPath);
};

extern std::vector<Trajectory> trajectories;