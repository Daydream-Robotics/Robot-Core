#include "api.h"
#include "brain/communication/nanoLink.hpp"
#include "brain/control/driverInputReader.hpp"

#include "pros/device.hpp"
#include "pros/error.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <mutex>
#include <vector>

namespace brain {
namespace {

// TODO: Move to constants file
constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RPM_TO_RAD_S = PI / 30.0;
const char* const RUMBLE[] = {".", "-", "..", "--", ".-.", "-.-"};

/**
 * @brief Maps a validated wire gearset to its PROS equivalent.
 * @param value Gearset value from GroupCfg.
 * @return pros::MotorGears Corresponding motor cartridge.
 */
pros::MotorGears gears(uint8_t value) {
    if (value == static_cast<uint8_t>(packets::Gearset::RED)) return pros::MotorGears::red;
    if (value == static_cast<uint8_t>(packets::Gearset::BLUE)) return pros::MotorGears::blue;
    return pros::MotorGears::green;
}
/**
 * @brief Maps a validated wire brake mode to its PROS equivalent.
 * @param value BrakeMode value from GroupCfg.
 * @return pros::MotorBrake Corresponding PROS brake mode.
 */
pros::MotorBrake brake(uint8_t value) {
    if (value == static_cast<uint8_t>(packets::BrakeMode::BRAKE)) return pros::MotorBrake::brake;
    if (value == static_cast<uint8_t>(packets::BrakeMode::HOLD)) return pros::MotorBrake::hold;
    return pros::MotorBrake::coast;
}
}

NanoLink::NanoLink(pros::Controller& controller,
                   const std::vector<pros::MotorGroup*>& groups,
                   const std::vector<pros::Rotation*>& wheels,
                   const std::array<pros::adi::DigitalOut*, packets::MAX_ADI>& outputs)
    : controller_(controller), outputs_(outputs) {
    for (auto* motor : groups) { Group group; group.motor = motor; groups_.push_back(std::move(group)); }
    for (auto* sensor : wheels) { Wheel wheel; wheel.sensor = sensor; wheels_.push_back(std::move(wheel)); }
}
NanoLink::~NanoLink() = default;

bool NanoLink::bindingsValid() const {
    if (groups_.empty() || groups_.size() > packets::MAX_GROUPS || wheels_.size() > packets::MAX_ROTATIONS) return false;
    for (const auto& group : groups_) if (!group.motor) return false;
    for (const auto& wheel : wheels_) if (!wheel.sensor) return false;
    return true;
}

packets::Error NanoLink::applyConfiguration(const BrainConfiguration& config) {
    std::lock_guard<pros::Mutex> lock(mutex_);
    const auto& fixed = config.fixed;
    if (!fixed.nGroups || fixed.nGroups != groups_.size() || fixed.nRotations != wheels_.size() ||
        fixed.nGroups > packets::MAX_GROUPS || fixed.nRotations > packets::MAX_ROTATIONS) return packets::Error::GROUP_COUNT;
    if (!fixed.adiExpanderPort && (fixed.adiOutEnable & 0xff00)) return packets::Error::CFG_PORT;
    // CONFIG may change settings, but device replacement belongs to the owner.
    for (size_t i = 0; i < groups_.size(); ++i) {
        const auto& cfg = config.motorGroups[i];
        if (!groups_[i].motor || !cfg.nMotors || cfg.nMotors > packets::MAX_MOTORS || cfg.gearset > 2 || cfg.brakeMode > 2) return packets::Error::RANGE;
        const auto ports = groups_[i].motor->get_port_all();
        if (ports.size() != cfg.nMotors || !std::equal(ports.begin(), ports.end(), cfg.ports)) return packets::Error::CFG_PORT;
    }
    for (size_t i = 0; i < wheels_.size(); ++i)
        if (!wheels_[i].sensor || wheels_[i].sensor->get_port() != config.trackingWheels[i].port) return packets::Error::CFG_PORT;
    // Check both the ADI slot and smart port; a non-null output may still refer to the wrong device.
    for (int i = 0; i < packets::MAX_ADI; ++i) {
        if (!(fixed.adiOutEnable & (1u << i))) continue;
        if (!outputs_[i]) return packets::Error::CFG_PORT;
        const auto port = outputs_[i]->get_port();
        const auto smart = std::get<0>(port);
        const auto adi = std::get<1>(port);
        const int slot = i % 8;
        const bool validAdi = adi == slot + 1 || adi == 'A' + slot || adi == 'a' + slot;
        const int expectedSmart = i < 8 ? INTERNAL_ADI_PORT : fixed.adiExpanderPort;
        if (!validAdi || smart != expectedSmart) return packets::Error::CFG_PORT;
    }

    // Stop the existing groups before changing settings or resetting accumulated positions.
    brakeGroupsUnlocked();
    for (size_t i = 0; i < groups_.size(); ++i) {
        auto& group = groups_[i];
        const auto& cfg = config.motorGroups[i];
        group.gear = gears(cfg.gearset); group.brakeMode = brake(cfg.brakeMode);
        group.last.assign(cfg.nMotors, 0.0); group.valid.assign(cfg.nMotors, false);
        group.position = 0; group.missing = false;
        for (size_t j = 0; j < cfg.nMotors; ++j)
            if (pros::Device::get_plugged_type(std::abs(cfg.ports[j])) != pros::DeviceType::motor) group.missing = true;
        if (!group.missing && (group.motor->set_gearing_all(group.gear) == PROS_ERR ||
            group.motor->set_encoder_units_all(pros::MotorEncoderUnits::degrees) == PROS_ERR ||
            group.motor->set_brake_mode_all(group.brakeMode) == PROS_ERR)) return packets::Error::REQ_DEVICE;
    }
    for (size_t i = 0; i < wheels_.size(); ++i) {
        auto& wheel = wheels_[i];
        wheel.reversed = (config.trackingWheels[i].flags & 1) != 0;
        wheel.missing = pros::Device::get_plugged_type(config.trackingWheels[i].port) != pros::DeviceType::rotation;
        wheel.valid = false; wheel.last = 0; wheel.position = 0;
        if (!wheel.missing && wheel.sensor->set_reversed(wheel.reversed) == PROS_ERR) return packets::Error::REQ_DEVICE;
    }
    // Retain levels only for outputs still on the same physical pins.
    uint16_t keep = enabled_ & fixed.adiOutEnable;
    if (expander_ != fixed.adiExpanderPort) keep &= 0xff;
    uint16_t levels = (keep & level_) | (~keep & (fixed.adiInitial ^ fixed.adiActiveLow) & fixed.adiOutEnable);
    const bool expanderPresent = fixed.adiExpanderPort && pros::Device::get_plugged_type(fixed.adiExpanderPort) == pros::DeviceType::adi;
    for (int i = 0; i < packets::MAX_ADI; ++i) {
        if ((fixed.adiOutEnable & (1u << i)) && (i < 8 || expanderPresent) &&
            outputs_[i]->set_value((levels & (1u << i)) != 0) == PROS_ERR) return packets::Error::REQ_DEVICE;
    }
    enabled_ = fixed.adiOutEnable; activeLow_ = fixed.adiActiveLow; level_ = levels; expander_ = fixed.adiExpanderPort;
    expanderMissing_ = fixed.adiExpanderPort && !expanderPresent;
    return packets::Error::NONE;
}

BrainState NanoLink::sample() {
    std::lock_guard<pros::Mutex> lock(mutex_);
    BrainState result{};
    if (!bindingsValid()) return result;
    result.driver = DriverInputReader(controller_).read();
    int battery = pros::battery::get_voltage();
    result.batteryVoltage = battery == PROS_ERR ? 0 : static_cast<uint16_t>(battery);
    if (pros::competition::is_disabled()) result.statusFlags |= packets::status::DISABLED;
    if (pros::competition::is_autonomous()) result.statusFlags |= packets::status::AUTON;
    if (pros::competition::is_connected()) result.statusFlags |= packets::status::COMP;
    if (pros::competition::is_field_control()) result.statusFlags |= packets::status::FIELD;
    result.sensors.sampledAt = static_cast<uint32_t>(pros::c::micros());
    result.sensors.motorGroupCount = static_cast<uint8_t>(groups_.size());
    result.sensors.trackingWheelCount = static_cast<uint8_t>(wheels_.size());
    for (size_t i = 0; i < groups_.size(); ++i) {
        auto& g = groups_[i]; auto& out = result.sensors.motorGroups[i];
        bool missing = false;
        for (int8_t port : g.motor->get_port_all()) if (pros::Device::get_plugged_type(std::abs(port)) != pros::DeviceType::motor) missing = true;
        if (g.missing && !missing) { g.motor->set_gearing_all(g.gear); g.motor->set_encoder_units_all(pros::MotorEncoderUnits::degrees); g.motor->set_brake_mode_all(g.brakeMode); }
        g.missing = missing;
        // Average changes from motors with consecutive valid readings; a reconnect re-seeds the baseline.
        auto positions = g.motor->get_position_all(); double change = 0; int count = 0;
        for (size_t j = 0; j < g.last.size(); ++j) {
            bool valid = j < positions.size() && positions[j] != PROS_ERR_F && std::isfinite(positions[j]);
            if (valid && g.valid[j]) { change += (positions[j] - g.last[j]) * DEG_TO_RAD; ++count; }
            if (valid) g.last[j] = positions[j]; else missing = true;
            g.valid[j] = valid;
        }
        if (count) g.position += change / count;
        out.positionRad = g.position;
        double velocity = 0; count = 0;
        for (double rpm : g.motor->get_actual_velocity_all()) if (rpm != PROS_ERR_F && std::isfinite(rpm)) { velocity += rpm; ++count; }
        out.velocity = count ? static_cast<float>(velocity / count * RPM_TO_RAD_S) : 0;
        for (uint32_t flags : g.motor->get_faults_all()) {
            if (flags == static_cast<uint32_t>(PROS_ERR)) missing = true;
            else out.faultFlags |= static_cast<uint8_t>(flags & 0x0f);
        }
        if (missing) out.faultFlags |= packets::fault::MISSING;
    }
    for (size_t i = 0; i < wheels_.size(); ++i) {
        auto& w = wheels_[i]; auto& out = result.sensors.trackingWheels[i];
        bool missing = pros::Device::get_plugged_type(w.sensor->get_port()) != pros::DeviceType::rotation;
        if (w.missing && !missing) w.sensor->set_reversed(w.reversed);
        w.missing = missing;
        // Hold accumulated position across a failed read and re-seed on the next good reading.
        int32_t raw = w.sensor->get_position(); bool valid = raw != PROS_ERR && !missing;
        if (valid) { double rad = raw * DEG_TO_RAD / 100.0; if (w.valid) w.position += rad - w.last; w.last = rad; }
        w.valid = valid; out.positionRad = w.position;
        if (!valid) out.faultFlags = packets::fault::MISSING;
    }
    if (expander_) {
        bool missing = pros::Device::get_plugged_type(expander_) != pros::DeviceType::adi;
        if (expanderMissing_ && !missing) for (int i = 8; i < packets::MAX_ADI; ++i) if (outputs_[i]) outputs_[i]->set_value((level_ & (1u << i)) != 0);
        expanderMissing_ = missing;
        if (missing) result.statusFlags |= packets::status::NO_EXPANDER;
    }
    return result;
}

packets::Error NanoLink::applyOutputs(const NanoCommand& command) {
    std::lock_guard<pros::Mutex> lock(mutex_);
    if (!bindingsValid()) return packets::Error::CFG_PORT;
    if (command.fixed.nGroups != groups_.size()) return packets::Error::GROUP_COUNT;
    for (size_t i = 0; i < groups_.size(); ++i) {
        const auto& cmd = command.groups[i]; auto& motor = *groups_[i].motor;
        int32_t rc;
        switch (static_cast<packets::GroupMode>(cmd.mode)) {
        case packets::GroupMode::VOLTAGE: rc = motor.move_voltage(std::lround(std::clamp(cmd.value, -12000.0f, 12000.0f))); break;
        case packets::GroupMode::VELOCITY: rc = motor.move_velocity(std::lround(std::clamp(static_cast<double>(cmd.value), static_cast<double>(std::numeric_limits<int32_t>::min()), static_cast<double>(std::numeric_limits<int32_t>::max())))); break;
        case packets::GroupMode::POSITION: rc = motor.move_absolute(cmd.value, cmd.aux); break;
        default: rc = motor.brake(); break;
        }
        if (rc == PROS_ERR) { brakeGroupsUnlocked(); return packets::Error::REQ_DEVICE; }
    }
    // Only configured, masked outputs change; convert logical values to physical levels.
    const uint16_t mask = command.fixed.adiOutMask & enabled_;
    for (int i = 0; i < packets::MAX_ADI; ++i) if (mask & (1u << i)) {
        bool level = ((command.fixed.adiOutState ^ activeLow_) & (1u << i)) != 0;
        if (outputs_[i]->set_value(level) == PROS_ERR) { brakeGroupsUnlocked(); return packets::Error::REQ_DEVICE; }
        if (level) level_ |= 1u << i; else level_ &= ~(1u << i);
    }
    return packets::Error::NONE;
}

packets::Error NanoLink::executeRequest(const packets::CommandFixed& cmd) {
    std::lock_guard<pros::Mutex> lock(mutex_);
    if (!bindingsValid()) return packets::Error::CFG_PORT;
    switch (static_cast<packets::Request>(cmd.requestType)) {
    case packets::Request::NONE: return packets::Error::NONE;
    case packets::Request::GROUP_TARE: {
        if (cmd.requestTarget >= groups_.size()) return packets::Error::REQ_TARGET;
        auto& group = groups_[cmd.requestTarget];
        if (group.motor->tare_position_all() == PROS_ERR) return packets::Error::REQ_DEVICE;
        // A tare changes raw encoder positions, so re-seed without reporting it as movement.
        std::fill(group.valid.begin(), group.valid.end(), false);
        return packets::Error::NONE;
    }
    case packets::Request::CONTROLLER_RUMBLE:
        if (!std::isfinite(cmd.requestValue) || cmd.requestValue < 0 || cmd.requestValue >= std::size(RUMBLE)) return packets::Error::RANGE;
        return controller_.rumble(RUMBLE[static_cast<size_t>(cmd.requestValue)]) == PROS_ERR ? packets::Error::REQ_DEVICE : packets::Error::NONE;
    default: return packets::Error::REQ_TYPE;
    }
}

void NanoLink::brakeGroupsUnlocked() { for (auto& g : groups_) if (g.motor) g.motor->brake(); }

void NanoLink::brakeGroups() {
    std::lock_guard<pros::Mutex> lock(mutex_);
    brakeGroupsUnlocked();
}
} // namespace brain
