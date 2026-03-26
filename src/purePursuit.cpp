#include "purePursuit.hpp"
#include "helpers.hpp"
#include <cmath>
#include "odometry.hpp"
#include "subsystems.h"

#include "main.h"


PurePursuit::PurePursuit(std::vector<Position> path) 
    : velocityPID(PP_KP, PP_KI, PP_KD, 0.0) {
    this->path = path;
}

void PurePursuit::setPath(std::vector<TargetPoint> new_path) {
    path.clear();
    for (const auto& point : new_path) {
        path.push_back({point.x, point.y});
    }
    lastPassedPointIndex = 0;
}


bool PurePursuit::step() {
    odom.updatePose();

    double cur_x = odom.pos_x;
    double cur_y = odom.pos_y;
    double cur_heading_deg = odom.getYaw();
    
    if (calcDistBetweenPoints({cur_x, cur_y}, path.back()) < END_TOLERANCE) {
        leftMotors.move_velocity(0);
        rightMotors.move_velocity(0);
        return true;
    }

    Position target_point = findLookaheadPoint({cur_x, cur_y});
    Position local_target_coord = convertToRobotCoords({cur_x, cur_y}, cur_heading_deg, target_point);
    pros::lcd::print(6, "LX %.2lf, LY %.2lf", local_target_coord.x, local_target_coord.y);

    // Using actual distance to target ensures accurate curvature regardless of point sparsity
    double actual_dist = calcDistBetweenPoints({cur_x, cur_y}, target_point);
    double curvature = 0.0;
    if (actual_dist > 0.01) {
        curvature = (2.0 * local_target_coord.x) / (actual_dist * actual_dist);
    }
    pros::lcd::print(1, "Cur: %lf", curvature);

    double radius = 0; // not used (may be used later)
    if (std::abs(curvature) > 0.001) {
        radius = 1 / curvature;
    }

    // set up pid here later
    int base_vel = 50; // this will be changed 

    int right_vel = base_vel - (curvature * TURN_RATE);
    int left_vel = base_vel + (curvature * TURN_RATE);

    rightMotors.move_velocity(right_vel);
    leftMotors.move_velocity(left_vel);

    return false;
}


Position PurePursuit::convertToRobotCoords(Position robot_pos, double robot_heading_deg, Position target_point) {
    double dx = target_point.x - robot_pos.x;
    double dy = target_point.y - robot_pos.y;

    // localize displacements relative to orientation of robot
    double robot_heading_rad = convertDegToRad(robot_heading_deg);
    double localX = dx * cos(robot_heading_rad) + dy * sin(robot_heading_rad);
    double localY = dx * (-sin(robot_heading_rad)) + dy * cos(robot_heading_rad);

    return {localX, localY};
}


// TODO: set up early exit when it's clear no point is going to be closer
int PurePursuit::findClosestPointIndex(Position robot_position) {
    int closest_index = 0;
    std::optional<double> min_dist = std::nullopt;

    for (int i = 0; i < this->path.size(); i++) {
        Position point = path[i];

        double point_dist = calcDistBetweenPoints(robot_position, point);

        if (!min_dist.has_value()) {
            min_dist = point_dist;
        } else if (point_dist < min_dist.value()) {
            min_dist = point_dist;
            closest_index = i;
        }

    }

    return closest_index;
}


Position PurePursuit::findLookaheadPoint(Position robot_position) {
    int start_point_index = findClosestPointIndex(robot_position);

    if (path.empty()) {
        pros::lcd::print(7 ,"NO PATH ADDED");
        return {0, 0};
    }

    // Update the last passed point to prevent tracking backwards
    lastPassedPointIndex = start_point_index;
    // lastPassedPointIndex = std::max(lastPassedPointIndex, start_point_index);

    for (int i = lastPassedPointIndex; i < path.size(); i++) {
        Position iter_point = path[i];
        double dist_to_point = calcDistBetweenPoints(iter_point, robot_position);

        // Find the first point that sits outside the lookahead distance radius
        if (dist_to_point >= look_ahead_dist) {
            return iter_point;
        }
    }

    // If no point is far enough ahead, default to the very last point
    return path.back();
}