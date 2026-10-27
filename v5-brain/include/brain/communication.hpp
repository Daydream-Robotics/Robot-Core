#pragma once

#include "brain/types.hpp"
#include "robotConfig.hpp"
#include "serial/packets.hpp"
#include "serial/serialProtocol.hpp"
#include "pros/rtos.hpp"

#include <array>
#include <cstdint>
#include <memory>

namespace brain {

struct BrainConfiguration {
    shared::RobotConfig robot{};
    packets::ConfigFixed fixed{};
    std::array<packets::GroupCfg, packets::MAX_GROUPS> motorGroups{};
    std::array<packets::RotationCfg, packets::MAX_ROTATIONS> trackingWheels{};
};

struct LinkExchange {
    bool commandReceived = false;
    NanoCommand command{};
    packets::Error error = packets::Error::NONE;
};

class BrainStateMailbox {
public:
    void publish(const BrainState& state);
    bool read(BrainState& state) const;

private:
    mutable pros::Mutex mutex_;
    BrainState state_{};
    bool hasValue_ = false;
};

class NanoCommandMailbox {
public:
    void publish(const NanoCommand& command);
    bool read(NanoCommand& command) const;

private:
    mutable pros::Mutex mutex_;
    NanoCommand command_{};
    bool hasValue_ = false;
};

class NanoLink {
public:
    explicit NanoLink(SerialProtocol& serial);

    bool configure(const BrainConfiguration& config);
    LinkExchange exchange(const BrainState& state);

private:
    SerialProtocol& serial_;
};

class CommunicationTask {
public:
    CommunicationTask(NanoLink& link,
                      BrainStateMailbox& states,
                      NanoCommandMailbox& commands);

    void start();
    void stop();

private:
    void run();

    NanoLink& link_;
    BrainStateMailbox& states_;
    NanoCommandMailbox& commands_;
    std::unique_ptr<pros::Task> task_;
};

} // namespace brain
