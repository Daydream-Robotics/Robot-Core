#include "autonomous/autonomous.hpp"
#include "config/constants.h"
#include <iostream>

Autonomous::Autonomous(PathFollower& pathFollower, const SerialLink& link, const std::vector<ALS_Path>& paths)
    : m_pathFollower(pathFollower), m_link(link), m_paths(paths), m_ticker(MPC_SAMPLE_PERIOD) {}

bool Autonomous::enabled() const {
    const std::optional<SerialLink::Sample> sample = m_link.latestSample();
    return sample && m_link.linkUp() &&
           (sample->status & packets::status::AUTON) && !(sample->status & packets::status::DISABLED);
}

void Autonomous::holdStopped(bool whileEnabled) {
    while (!shutdownRequested() && enabled() == whileEnabled) {
        m_pathFollower.stop();
        m_ticker.wait();
    }
}

bool Autonomous::runPath(std::size_t index) {
    std::cerr << "Path " << index << " started\n";
    // JerryIO paths have no direction, so every path runs forwards
    m_pathFollower.setPath(m_paths[index], PathFlag::FORWARDS);
    while (!m_pathFollower.step()) {
        m_ticker.wait();
        if (shutdownRequested() || !enabled()) {
            std::cerr << "Path " << index << " aborted\n";
            return false;
        }
    }
    std::cerr << "Path " << index << " done\n";
    return true;
}

void Autonomous::run() {
    m_pathFollower.stop();
    while (!shutdownRequested()) {
        // outside an enabled autonomous the brain or the field owns the drive
        holdStopped(false);
        if (shutdownRequested()) {
            break;
        }

        std::cerr << "Autonomous started\n";
        for (std::size_t i = 0; i < m_paths.size(); i++) {
            if (!runPath(i)) {
                break;
            }
        }

        // hold still until this autonomous period ends
        holdStopped(true);
        std::cerr << "Autonomous ended\n";
    }
    m_pathFollower.stop();
}
