#include "autonomous/autonomous.hpp"
#include "config/constants.h"
#include <iostream>

Autonomous::Autonomous(PathFollower& pathFollower, const SerialLink& link, const std::vector<ALS_Path>& paths, const std::vector<PathFlag>& flags)
    : m_pathFollower(pathFollower), m_link(link), m_paths(paths), m_flags(flags), m_ticker(MPC_SAMPLE_PERIOD) {}

bool Autonomous::enabled() const {
    const std::optional<SerialLink::Sample> sample = m_link.latestSample();
    return sample && m_link.linkUp() &&
           (sample->status & packets::status::AUTON) && !(sample->status & packets::status::DISABLED);
}

bool Autonomous::autonOver() {
    if (shutdownRequested()) {
        return true;
    }
    // no state from the brain: hold and keep the progress.
    if (!m_link.linkUp()) {
        if (!m_linkLost) {
            std::cerr << "Brain link lost, holding\n";
            m_linkLost = true;
        }
        return false;
    }
    if (m_linkLost) {
        std::cerr << "Brain link back\n";
        m_linkLost = false;
    }
    // over unless the latest sample says enabled autonomous
    const std::optional<SerialLink::Sample> sample = m_link.latestSample();
    return !sample || !(sample->status & packets::status::AUTON) || (sample->status & packets::status::DISABLED);
}

void Autonomous::waitForAuton() {
    while (!shutdownRequested() && !enabled()) {
        m_pathFollower.stop();
        m_ticker.wait();
    }
}

void Autonomous::waitForAutonEnd() {
    while (!autonOver()) {
        m_pathFollower.stop();
        m_ticker.wait();
    }
}

bool Autonomous::runPath(std::size_t index) {
    const PathFlag flag = index < m_flags.size() ? m_flags[index] : PathFlag::FORWARDS;
    std::cerr << "Path " << index << " started" << (flag == PathFlag::REVERSE ? " (reverse)" : "") << "\n";
    m_pathFollower.setPath(m_paths[index], flag);
    // while the link is down step() holds the drive stopped and keeps the path progress
    while (!m_pathFollower.step()) {
        m_ticker.wait();
        if (autonOver()) {
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
        waitForAuton();
        if (shutdownRequested()) {
            break;
        }

        std::cerr << "Autonomous started\n";
        m_linkLost = false;
        for (std::size_t i = 0; i < m_paths.size(); i++) {
            if (!runPath(i)) {
                break;
            }
        }

        // hold still until this autonomous period ends
        waitForAutonEnd();
        std::cerr << "Autonomous ended\n";
    }
    m_pathFollower.stop();
}
