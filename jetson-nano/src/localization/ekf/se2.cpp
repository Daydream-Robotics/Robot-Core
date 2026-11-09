#include "localization/ekf/se2.hpp"

#include "utils/mathHelpers.hpp"

#include <cmath>

namespace se2 {

Pose compose(const Pose& a, const Pose& b) {
    // add the headings and wrap
    double heading = utils::wrapAngle(a.theta + b.theta);

    // cos and sin of a's heading
    double cosA = std::cos(a.theta);
    double sinA = std::sin(a.theta);

    // rotate b's x and y into the world frame
    double dxWorld = b.x * cosA - b.y * sinA;
    double dyWorld = b.x * sinA + b.y * cosA;

    // add the rotated move to a's position
    double x = a.x + dxWorld;
    double y = a.y + dyWorld;

    return Pose{x, y, heading};
}

Pose inverse(const Pose& a) {
    // negate a's heading and wrap
    double heading = utils::wrapAngle(-a.theta);

    // negate a's position
    double backX = -a.x;
    double backY = -a.y;

    // cos and sin of a's heading
    double cosA = std::cos(a.theta);
    double sinA = std::sin(a.theta);

    // rotate the negated position by -a.theta into a's frame
    double x = backX * cosA + backY * sinA;
    double y = -backX * sinA + backY * cosA;

    return Pose{x, y, heading};
}

Pose between(const Pose& a, const Pose& b) {
    return compose(inverse(a), b);
}

Eigen::Matrix3d composeJacA(const Pose& a, const Pose& b) {
    // cos and sin of a's heading
    double cosA = std::cos(a.theta);
    double sinA = std::sin(a.theta);

    // rotate b's x and y into the world frame
    double dxWorld = b.x * cosA - b.y * sinA;
    double dyWorld = b.x * sinA + b.y * cosA;

    // start from identity
    Eigen::Matrix3d jac = Eigen::Matrix3d::Identity();

    // fill the a.theta column for x and y
    jac(0, 2) = -dyWorld;
    jac(1, 2) = dxWorld;

    return jac;
}

Eigen::Matrix3d composeJacB(const Pose& a) {
    // cos and sin of a's heading
    double cosA = std::cos(a.theta);
    double sinA = std::sin(a.theta);

    // start from identity
    Eigen::Matrix3d jac = Eigen::Matrix3d::Identity();

    // put the rotation by a's heading in the top-left 2x2
    jac(0, 0) = cosA;
    jac(0, 1) = -sinA;
    jac(1, 0) = sinA;
    jac(1, 1) = cosA;

    return jac;
}

} 
