#include "localization/ekf/se2.hpp"

#include "utils/mathHelpers.hpp"

#include <cmath>

namespace se2 {

Pose compose(const Pose& a, const Pose& b) {
    // turns add, wrapped back into (-pi, pi]
    double heading = utils::wrapAngle(a.theta + b.theta);

    // rotate by a's heading, the direction the robot faced before the move
    double cosA = std::cos(a.theta);
    double sinA = std::sin(a.theta);

    // b's forward/left move turned into world-frame x and y changes
    double dxWorld = b.x * cosA - b.y * sinA;
    double dyWorld = b.x * sinA + b.y * cosA;

    // start at a's position, then apply the move
    double x = a.x + dxWorld;
    double y = a.y + dyWorld;

    return Pose{x, y, heading};
}

Pose inverse(const Pose& a) {
    // turn back by a's heading, wrapped into (-pi, pi]
    double heading = utils::wrapAngle(-a.theta);

    // the move from a back to the origin, in world-frame x and y
    double backX = -a.x;
    double backY = -a.y;

    // rotate by -a.theta, written with cos/sin of a.theta (cos(-t) = cos(t), sin(-t) = -sin(t))
    double cosA = std::cos(a.theta);
    double sinA = std::sin(a.theta);

    // the move back turned into a's frame (forward/left as seen from a)
    double x = backX * cosA + backY * sinA;
    double y = -backX * sinA + backY * cosA;

    return Pose{x, y, heading};
}

Pose between(const Pose& a, const Pose& b) {
    return compose(inverse(a), b);
}

Eigen::Matrix3d composeJacA(const Pose& a, const Pose& b) {
    // TODO: identity plus the theta column from rotating b's translation by a.theta
    Eigen::Matrix3d jac = Eigen::Matrix3d::Identity();
    return jac;
}

Eigen::Matrix3d composeJacB(const Pose& a) {
    // TODO: a.theta rotation in the top-left 2x2, 1 in the bottom-right
    Eigen::Matrix3d jac = Eigen::Matrix3d::Identity();
    return jac;
}

} 
