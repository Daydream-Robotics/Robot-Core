#include <chrono>
#include <iostream>
#include <optional>
#include <thread>
#include <vector>

#include "autonomous/autonomous.hpp"
#include "config/constants.h"
#include "localization/odometry.hpp"
#include "motion/control/mpc.hpp"
#include "motion/pathFollower.hpp"
#include "motion/pathing/paths.hpp"
#include "protocol/serial.hpp"
#include "utils/helpers.hpp"

// The Nano runs everything: it reads the brain's STATE through SerialLink, runs the path
// following controller and sends drive commands back in COMMAND. the brain only does I/O.

/**
 * @brief the robot's latest pose, for the path follower
 *
 * the pose estimate (EKF) runs on the Nano but isn't part of this program yet.
 * replace the body with the EKF's latest pose: same frame as Pose (+x forward, +y left,
 * theta 0 along +y and pi/2 along +x) and the same field frame as the JerryIO paths
 * (seeded from the start pose Path::buildAllPathsFromJerryIO returns).
 * until then it returns nothing and the robot holds still
 *
 * @returns latest pose, or nothing if there isn't a good one
 */
 // TODO: remove this when EKF is addded
static std::optional<Pose> latestPose() {
    return std::nullopt;
}

int main() {
    // link and path file settings are in config/constants.h ([NANO])
    Pose startPose{0.0, 0.0, 0.0};
    std::vector<PathFlag> pathFlags;
    const std::vector<ALS_Path> paths = Path::buildAllPathsFromJerryIO(NANO_PATH_FILE, startPose, pathFlags);
    if (paths.empty()) {
        std::cerr << "No paths loaded from " << NANO_PATH_FILE << "\n";
        return 1;
    }

    const SerialProtocol::Transport transport = NANO_LINK_USE_USB ? SerialProtocol::Transport::USB : SerialProtocol::Transport::RS485;
    SerialLink link(NANO_LINK_DEVICE, NANO_LINK_BAUD, NANO_DE_GPIO, packets::buildLinkConfig(), transport);

    // motion controllers (tuning in config/constants.h).
    MPC mpc;
    PathFollower pathFollower(mpc, link, latestPose);
    Autonomous autonomous(pathFollower, link, paths, pathFlags);

    if (!link.start()) {
        std::cerr << "Could not start the brain link\n";
        return 1;
    }
    installShutdownHandlers();

    autonomous.run();

    // give the link a few ticks to send the final stop
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return 0;
}
