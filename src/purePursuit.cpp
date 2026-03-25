#include "purePursuit.hpp"
#include "helpers.hpp"
#include <cmath>
#include "odometry.hpp"


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

    double dx = target_point.x - cur_x;
    double dy = target_point.y - cur_y;

    // localize displacements relative to orientation of robot
    double cur_heading_rad = convertDegToRad(cur_heading_deg);
    double localX = dx * cos(cur_heading_rad) + dy * sin(cur_heading_rad);
    double localY = dx * (-sin(cur_heading_rad)) + dy * cos(cur_heading_rad);

    double curvature = (2.0 * localY) / pow(look_ahead_dist, 2.0);

    // set up pid here later

    int base_vel = 50 // this will be changed 
    
}

Position PurePursuit::findLookaheadPoint(Position position) {

}