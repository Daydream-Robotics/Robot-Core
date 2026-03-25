#include "purePursuit.hpp"
#include "helpers.hpp"
#include <cmath>
#include "odometry.hpp"

#include "main.h"


PurePursuit::PurePursuit(std::vector<Position> path) 
    : velocityPID(PP_KP, PP_KI, PP_KD, 0.0) {
    this->path = path;
}


std::pair<double, double> PurePursuit::step() {
    odom.updatePose();

    double cur_x = odom.pos_x;
    double cur_y = odom.pos_y;
    double cur_heading_deg = odom.getYaw();

    Position target_point = findLookaheadPoint({cur_x, cur_y});
    Position local_target_coord = convertToRobotCoords({cur_x, cur_y}, cur_heading_deg, target_point);

    double curvature = (2.0 * local_target_coord.y) / pow(look_ahead_dist, 2.0);

    // set up pid here later

    int base_vel = 50; // this will be changed 

    int right_vel = 0;
    int left_vel = 0;

    pros::delay(10);
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


double calcDistBetweenPoints(Position pt1, Position pt2) {
    return std::sqrt(std::pow(pt1.x - pt2.x, 2) + std::pow(pt1.y - pt2.y, 2));
}


// TODO: set up early exit when it's clear no point is going to be closer
int PurePursuit::findClosestPointIndex(Position cur_position) {
    int closest_index = 0;
    double min_dist = 0;

    for (int i = 0; i < this->path.size(); i++) {
        Position point = path[i];

        double point_dist = calcDistBetweenPoints(cur_position, point);
        if (point_dist < min_dist) {
            min_dist = point_dist;
            closest_index = i;
        }
    }

    return closest_index;
}


Position PurePursuit::findLookaheadPoint(Position cur_position) {
    int start_point_index = findClosestPointIndex(cur_position);

    Position point_to_cast_to;
    for (int i = start_point_index; i < this->path.size(); i++) {
        Position iter_point = path[i];
        
        double dist_to_point = calcDistBetweenPoints(iter_point, cur_position);
        if (dist_to_point > this->look_ahead_dist) {
            point_to_cast_to = iter_point;
            break;
        }
    }

    Position 


}