#include "api.h"
#include "protocol/serial.hpp"
#include <cmath>
#include <cstring>
#include <mutex>
#include <utility>
#include <cstdarg>
#include <cstdio>
#include <algorithm>

SerialLink::SerialLink(brain::NanoLink& hw, std::vector<uint8_t> config, uint8_t port, int32_t baud, SerialProtocol::Transport transport)
    : hw_(hw), boot_(std::move(config)), port_(port), baud_(baud), transport_(transport) {}
SerialLink::~SerialLink() { stop(); }

bool SerialLink::start() {
    if (running_) return true;

    serial_.reset(new SerialProtocol());
    if (!(transport_ == SerialProtocol::Transport::USB ? serial_->setUpVexSerial() : serial_->setUpVexSerial(port_, baud_))) {
        serial_.reset(); return false;
    }
    cfg_ = {};
    cfg_.fixed.statePeriodMs = 10;
    cfg_.fixed.watchdogMs = 100;
    booted_ = true;
    haveCmd_ = false;
    seq_ = 1; lastCmd_ = 0; lastRequest_ = 0; braked_ = false;
    history_ = {}; base_ = {}; cmdMs_ = pros::millis(); cmdUs_ = 0;
    error_ = boot_.size() <= packets::MAX_PAYLOAD && packets::sizeInRange(packets::CONFIG, static_cast<uint16_t>(boot_.size()))
        ? configFrame(boot_.data(), static_cast<uint16_t>(boot_.size())) : packets::Error::SIZE;
    running_ = true;
    reader_ = std::make_unique<pros::Task>([this] { readLoop(); }, transport_ == SerialProtocol::Transport::USB ? TASK_PRIORITY_MIN + 1 : TASK_PRIORITY_MAX - 2, TASK_STACK_DEPTH_DEFAULT, "nano reader");
    writer_ = std::make_unique<pros::Task>([this] { writeLoop(); }, TASK_PRIORITY_MAX - 2, TASK_STACK_DEPTH_DEFAULT, "nano writer");
    return true;
}

void SerialLink::stop() {
    running_ = false;
    // Do not delete a task while it holds the shared hardware/protocol lock.
    std::lock_guard<pros::Mutex> lock(mutex_);
    std::lock_guard<pros::Mutex> logLock(m_logMutex);
    if (reader_) { reader_->remove(); reader_.reset(); }
    if (writer_) { writer_->remove(); writer_.reset(); }
    hw_.brakeGroups();
    serial_.reset();
}

void SerialLink::setNanoControl(bool value) { nanoControl_ = value; }

packets::Error SerialLink::applyConfig(const brain::BrainConfiguration& cfg) {
    const auto& f = cfg.fixed;
    if (f.configId && f.configId == cfg_.fixed.configId) return packets::Error::NONE;
    if (!f.configId || f.statePeriodMs < 5 || f.statePeriodMs > 50 || f.watchdogMs < 20 || f.watchdogMs > 1000 || !f.nGroups || f.nGroups > packets::MAX_GROUPS || f.nRotations > packets::MAX_ROTATIONS) return packets::Error::RANGE;
    if (!f.adiExpanderPort && (f.adiOutEnable & 0xff00)) return packets::Error::CFG_PORT;
    bool used[22]{};
    if (transport_ == SerialProtocol::Transport::RS485 && port_ <= 21) used[port_] = true;
    auto claim = [&used](int port) {
        if (port < 1 || port > 21) return packets::Error::CFG_PORT;
        if (used[port]) return packets::Error::CFG_DUPLICATE;
        used[port] = true;
        return packets::Error::NONE;
    };
    if (f.adiExpanderPort) {
        auto error = claim(f.adiExpanderPort);
        if (error != packets::Error::NONE) return error;
    }
    unsigned totalMotors = 0;
    for (size_t i = 0; i < f.nGroups; ++i) {
        const auto& group = cfg.motorGroups[i];
        if (!group.nMotors || group.nMotors > packets::MAX_MOTORS || group.gearset > 2 || group.brakeMode > 2) return packets::Error::RANGE;
        totalMotors += group.nMotors;
        for (size_t j = 0; j < packets::MAX_MOTORS; ++j) {
            if (j >= group.nMotors) {
                if (group.ports[j]) return packets::Error::RANGE;
                continue;
            }
            auto error = claim(std::abs(static_cast<int>(group.ports[j])));
            if (error != packets::Error::NONE) return error;
        }
    }
    if (totalMotors > 21) return packets::Error::CFG_LIMIT;
    for (size_t i = 0; i < f.nRotations; ++i) {
        auto error = claim(cfg.trackingWheels[i].port);
        if (error != packets::Error::NONE) return error;
    }
    auto error = hw_.applyConfiguration(cfg);
    if (error != packets::Error::NONE) {
        hw_.brakeGroups();
        cfg_ = {};
        cfg_.fixed.statePeriodMs = 10;
        cfg_.fixed.watchdogMs = 100;
        serial_->setCommandGroups(0);
        base_ = {}; history_ = {}; haveCmd_ = false; lastCmd_ = 0;
        return error;
    }
    cfg_ = cfg;
    serial_->setCommandGroups(f.nGroups);
    base_ = {}; history_ = {}; haveCmd_ = false; braked_ = false; lastCmd_ = 0; cmdMs_ = pros::millis();
    return packets::Error::NONE;
}

packets::Error SerialLink::configFrame(const uint8_t* data, uint16_t size) {
    brain::BrainConfiguration cfg = cfg_;
    packets::PayloadReader reader{data, size};
    if (!reader.get(cfg.fixed) || cfg.fixed.nGroups > packets::MAX_GROUPS || cfg.fixed.nRotations > packets::MAX_ROTATIONS || size != packets::configSize(cfg.fixed.nGroups, cfg.fixed.nRotations)) return packets::Error::SIZE;
    for (size_t i = 0; i < cfg.fixed.nGroups; ++i) if (!reader.get(cfg.motorGroups[i])) return packets::Error::SIZE;
    for (size_t i = 0; i < cfg.fixed.nRotations; ++i) if (!reader.get(cfg.trackingWheels[i])) return packets::Error::SIZE;
    return applyConfig(cfg);
}

packets::Error SerialLink::commandFrame(const uint8_t* data, uint16_t size) {
    brain::NanoCommand cmd{};
    packets::PayloadReader reader{data, size};
    if (!reader.get(cmd.fixed)) return packets::Error::SIZE;
    if (!cfg_.fixed.configId) return packets::Error::NOT_CONFIGURED;
    if (cmd.fixed.configId != cfg_.fixed.configId) return packets::Error::CONFIG_ID;
    if (cmd.fixed.nGroups != cfg_.fixed.nGroups) return packets::Error::GROUP_COUNT;
    if (size != packets::commandSize(cmd.fixed.nGroups)) return packets::Error::SIZE;
    for (size_t i = 0; i < cmd.fixed.nGroups; ++i) {
        if (!reader.get(cmd.groups[i])) return packets::Error::SIZE;
        if (!std::isfinite(cmd.groups[i].value)) return packets::Error::RANGE;
    }
    cmd.valid = true;
    if (nanoControl_) { auto error = hw_.applyOutputs(cmd); if (error != packets::Error::NONE) return error; }
    cmdUs_ = static_cast<uint32_t>(pros::c::micros()); cmdMs_ = pros::millis();
    haveCmd_ = true; booted_ = false; braked_ = false; lastCmd_ = cmd.fixed.cmdSeq;
    for (const auto& snap : history_) if (snap.seq && snap.seq == cmd.fixed.ackStateSeq && (!base_.seq || static_cast<int16_t>(snap.seq - base_.seq) > 0)) { base_ = snap; break; }
    if (cmd.fixed.requestId && cmd.fixed.requestId != lastRequest_) {
        lastRequest_ = cmd.fixed.requestId;
        return hw_.executeRequest(cmd.fixed);
    }
    return packets::Error::NONE;
}

void SerialLink::readLoop() {
    while (running_) {
        while (auto frame = serial_->receive()) {
            std::lock_guard<pros::Mutex> lock(mutex_);
            auto error = frame->type == packets::CONFIG ? configFrame(frame->payload, frame->size) : commandFrame(frame->payload, frame->size);
            if (error != packets::Error::NONE) error_ = error;
        }
        pros::delay(1);
    }
}

void SerialLink::writeLoop() {
    uint32_t wake = pros::millis();
    while (running_) {
        uint8_t period;
        { std::lock_guard<pros::Mutex> lock(mutex_); period = cfg_.fixed.statePeriodMs; }
        pros::Task::delay_until(&wake, period);
        for (int i = 0; i < 2 && serial_->midFrame(); ++i) pros::delay(1);
        std::unique_lock<pros::Mutex> lock(mutex_);
        const bool watchdog = cfg_.fixed.configId && pros::millis() - cmdMs_ > cfg_.fixed.watchdogMs;
        if (watchdog && pros::competition::is_autonomous() && !braked_) { hw_.brakeGroups(); braked_ = true; }
        if (!nanoControl_ && !pros::competition::is_disabled()) { lock.unlock(); flushOneLog(); continue; }
        brain::BrainState state = hw_.sample();
        if (cfg_.fixed.configId && (state.sensors.motorGroupCount != cfg_.fixed.nGroups || state.sensors.trackingWheelCount != cfg_.fixed.nRotations)) { error_ = packets::Error::GROUP_COUNT; continue; }
        uint8_t payload[packets::MAX_PAYLOAD]{};
        packets::PayloadWriter writer{payload, sizeof(packets::StateFixed)};
        Snapshot snap; snap.seq = seq_;
        for (size_t i = 0; i < cfg_.fixed.nGroups; ++i) {
            const auto& s = state.sensors.motorGroups[i]; snap.group[i] = s.positionRad;
            writer.put(packets::GroupState{static_cast<float>(s.positionRad - base_.group[i]), s.velocity, s.faultFlags});
        }
        for (size_t i = 0; i < cfg_.fixed.nRotations; ++i) {
            const auto& s = state.sensors.trackingWheels[i]; snap.wheel[i] = s.positionRad;
            writer.put(packets::RotationState{static_cast<float>(s.positionRad - base_.wheel[i]), s.faultFlags});
        }
        packets::StateFixed f{};
        f.stateSeq = seq_; f.deltaBaseSeq = base_.seq; f.lastCmdSeq = lastCmd_; f.configId = cfg_.fixed.configId;
        f.lastRequestId = lastRequest_; f.lastError = static_cast<uint8_t>(error_);
        f.status = (state.statusFlags & ~(packets::status::WATCHDOG | packets::status::BOOTED | packets::status::CONTROLLER)) | (watchdog ? packets::status::WATCHDOG : 0) | (booted_ ? packets::status::BOOTED : 0) | (state.driver.connected ? packets::status::CONTROLLER : 0);
        f.batteryMv = state.batteryVoltage; std::memcpy(f.ctrlAxes, state.driver.axes.data(), sizeof f.ctrlAxes); f.ctrlButtons = state.driver.buttons;
        f.rxErrors = serial_->m_crcErrors + serial_->m_badFrames; f.nGroups = cfg_.fixed.nGroups; f.nRotations = cfg_.fixed.nRotations;
        const bool haveCmd = haveCmd_;
        const uint32_t cmdUs = cmdUs_;
        const packets::Error reportedError = error_;
        history_[seq_ % history_.size()] = snap;
        seq_ = seq_ == 0xffff ? 1 : seq_ + 1;
        lock.unlock();
        flushOneLog();
        f.tUs = static_cast<uint32_t>(pros::c::micros());
        f.cmdAgeUs = haveCmd ? f.tUs - cmdUs : packets::NO_COMMAND;
        std::memcpy(payload, &f, sizeof f);
        const bool sent = serial_->send(packets::STATE, writer);
        if (sent && reportedError != packets::Error::NONE) {
            std::lock_guard<pros::Mutex> guard(mutex_);
            if (error_ == reportedError) error_ = packets::Error::NONE;
        }
    }
}

void SerialLink::log(uint8_t level, const char* format, ...) {
    LogEntry entry;
    entry.level = level;
    // format into the entry
    va_list args;
    va_start(args, format);
    int length = std::vsnprintf(entry.text, sizeof(entry.text), format, args);
    va_end(args);
    // nothing to send
    if (length <= 0) {
        return;
    }
    // set the length
    entry.length = static_cast<uint8_t>(std::min<int>(length, packets::MAX_LOG_TEXT));

    // lock the queue so the writer can't take a line while this one is added
    std::lock_guard<pros::Mutex> lock(m_logMutex);
    // queue full so drop the line
    if (m_logCount == LOG_QUEUE_SIZE) {
        return;
    }
    // add to the back of the queue (ring)
    m_logQueue[(m_logHead + m_logCount) % LOG_QUEUE_SIZE] = entry;
    m_logCount++;
}


void SerialLink::flushOneLog() {
    // the line to send, copied out of the queue
    LogEntry entry;
    {
        // lock the queue while taking a line out
        std::lock_guard<pros::Mutex> lock(m_logMutex);
        // if queue empty return
        if (m_logCount == 0) {
            return;
        }
        // copy the oldest line
        entry = m_logQueue[m_logHead];
        // move the front of the queue to the next slot, wrapping at the end of the array
        m_logHead = (m_logHead + 1) % LOG_QUEUE_SIZE;
        // one less line queued
        m_logCount--;
    }
    // payload is 1 level byte, then the text without a terminator
    uint8_t payload[1 + packets::MAX_LOG_TEXT];
    // byte 0 is the level
    payload[0] = entry.level;
    // bytes 1 onward are the text
    std::memcpy(payload + 1, entry.text, entry.length);
    // send it as a log frame, 1 + text length bytes
    serial_->send(packets::LOG, payload, static_cast<uint16_t>(1 + entry.length));
}
