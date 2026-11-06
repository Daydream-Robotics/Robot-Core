#include "localization/ekf/se2.hpp"

#include "utils/mathHelpers.hpp"

#include <cmath>

namespace se2 {

Pose compose(const Pose& a, const Pose& b) {
    // TODO: rotate b's translation by a.theta, add to a's position, sum and wrap headings
    return Pose{0.0, 0.0, 0.0};
}

Pose inverse(const Pose& a) {
    // TODO: negate heading, rotate -a's translation by -a.theta
    return Pose{0.0, 0.0, 0.0};
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
