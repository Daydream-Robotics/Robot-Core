#include "protocol/serial.hpp"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <utility>

SerialLink::SerialLink(brain::NanoLink& hw, std::vector<uint8_t> config, uint8_t port, int32_t baud, SerialProtocol::Transport transport)
    : m_hw(hw), m_boot(std::move(config)), m_port(port), m_baud(baud), m_transport(transport) {}
SerialLink::~SerialLink() { stop(); }

bool SerialLink::start() {
    if (m_running) return true;
    if (!m_hw.bindingsValid()) return false;

    m_serial.reset(new SerialProtocol());
    if (!(m_transport == SerialProtocol::Transport::USB ? m_serial->setUpVexSerial() : m_serial->setUpVexSerial(m_port, m_baud))) {
        m_serial.reset(); return false;
    }
    // Start unconfigured so the Nano can supply a config if the boot payload is rejected.
    m_cfg = {};
    m_cfg.fixed.statePeriodMs = 10;
    m_cfg.fixed.watchdogMs = 100;
    m_booted = true;
    m_haveCmd = false;
    m_seq = 1; m_lastCmd = 0; m_lastRequest = 0; m_braked = false;
    m_history = {}; m_base = {}; m_cmdMs = pros::millis(); m_cmdUs = 0;
    m_error = m_boot.size() <= packets::MAX_PAYLOAD && packets::sizeInRange(packets::CONFIG, static_cast<uint16_t>(m_boot.size()))
        ? configFrame(m_boot.data(), static_cast<uint16_t>(m_boot.size())) : packets::Error::SIZE;
    m_running = true;
    m_reader = std::make_unique<pros::Task>([this] { readLoop(); }, m_transport == SerialProtocol::Transport::USB ? TASK_PRIORITY_MIN + 1 : TASK_PRIORITY_MAX - 2, TASK_STACK_DEPTH_DEFAULT, "nano reader");
    m_writer = std::make_unique<pros::Task>([this] { writeLoop(); }, TASK_PRIORITY_MAX - 2, TASK_STACK_DEPTH_DEFAULT, "nano writer");
    return true;
}

void SerialLink::stop() {
    m_running = false;
    // Do not delete a task while it holds the shared hardware/protocol lock.
    std::lock_guard<pros::Mutex> lock(m_mutex);
    std::lock_guard<pros::Mutex> logLock(m_logMutex);
    if (m_reader) { m_reader->remove(); m_reader.reset(); }
    if (m_writer) { m_writer->remove(); m_writer.reset(); }
    m_hw.brakeGroups();
    m_serial.reset();
}

void SerialLink::setNanoControl(bool value) { m_nanoControl = value; }

packets::Error SerialLink::applyConfig(const brain::BrainConfiguration& cfg) {
    const auto& f = cfg.fixed;
    if (f.configId && f.configId == m_cfg.fixed.configId) return packets::Error::NONE;
    if (!f.configId || f.statePeriodMs < 5 || f.statePeriodMs > 50 || f.watchdogMs < 20 || f.watchdogMs > 1000 || !f.nGroups || f.nGroups > packets::MAX_GROUPS || f.nRotations > packets::MAX_ROTATIONS) return packets::Error::RANGE;
    if (!f.adiExpanderPort && (f.adiOutEnable & 0xff00)) return packets::Error::CFG_PORT;
    // Reserve the serial port and reject duplicate smart-port assignments.
    bool used[22]{};
    if (m_transport == SerialProtocol::Transport::RS485 && m_port <= 21) used[m_port] = true;
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
    auto error = m_hw.applyConfiguration(cfg);
    if (error != packets::Error::NONE) {
        m_hw.brakeGroups();
        // Hardware settings may be partially applied; brake and return to an unconfigured state.
        m_cfg = {};
        m_cfg.fixed.statePeriodMs = 10;
        m_cfg.fixed.watchdogMs = 100;
        m_serial->setCommandGroups(0);
        m_base = {}; m_history = {}; m_haveCmd = false; m_lastCmd = 0;
        return error;
    }
    m_cfg = cfg;
    m_serial->setCommandGroups(f.nGroups);
    m_base = {}; m_history = {}; m_haveCmd = false; m_braked = false; m_lastCmd = 0; m_cmdMs = pros::millis();
    return packets::Error::NONE;
}

packets::Error SerialLink::configFrame(const uint8_t* data, uint16_t size) {
    brain::BrainConfiguration cfg = m_cfg;
    packets::PayloadReader reader{data, size};
    if (!reader.get(cfg.fixed) || cfg.fixed.nGroups > packets::MAX_GROUPS || cfg.fixed.nRotations > packets::MAX_ROTATIONS || size != packets::configSize(cfg.fixed.nGroups, cfg.fixed.nRotations)) return packets::Error::SIZE;
    for (size_t i = 0; i < cfg.fixed.nGroups; ++i) if (!reader.get(cfg.motorGroups[i])) return packets::Error::SIZE;
    for (size_t i = 0; i < cfg.fixed.nRotations; ++i) if (!reader.get(cfg.trackingWheels[i])) return packets::Error::SIZE;
    return applyConfig(cfg);
}

packets::Error SerialLink::commandFrame(const uint8_t* data, uint16_t size, uint32_t receivedUs, uint32_t receivedMs) {
    brain::NanoCommand cmd{};
    packets::PayloadReader reader{data, size};
    if (!reader.get(cmd.fixed)) return packets::Error::SIZE;
    if (!m_cfg.fixed.configId) return packets::Error::NOT_CONFIGURED;
    if (cmd.fixed.configId != m_cfg.fixed.configId) return packets::Error::CONFIG_ID;
    if (cmd.fixed.nGroups != m_cfg.fixed.nGroups) return packets::Error::GROUP_COUNT;
    if (size != packets::commandSize(cmd.fixed.nGroups)) return packets::Error::SIZE;
    for (size_t i = 0; i < cmd.fixed.nGroups; ++i) {
        if (!reader.get(cmd.groups[i])) return packets::Error::SIZE;
        if (!std::isfinite(cmd.groups[i].value)) return packets::Error::RANGE;
    }
    // Do not acknowledge the command or reset the watchdog if output application fails.
    cmd.valid = true;
    if (m_nanoControl) { auto error = m_hw.applyOutputs(cmd); if (error != packets::Error::NONE) return error; }
    m_cmdUs = receivedUs; m_cmdMs = receivedMs;
    m_haveCmd = true; m_booted = false; m_braked = false; m_lastCmd = cmd.fixed.cmdSeq;
    // Move the delta baseline only to a newer acknowledged snapshot still in history.
    for (const auto& snap : m_history) if (snap.seq && snap.seq == cmd.fixed.ackStateSeq && (!m_base.seq || static_cast<int16_t>(snap.seq - m_base.seq) > 0)) { m_base = snap; break; }
    if (cmd.fixed.requestId && cmd.fixed.requestId != m_lastRequest) {
        // Echo attempted requests even on error so the Nano does not execute them repeatedly.
        m_lastRequest = cmd.fixed.requestId;
        return m_hw.executeRequest(cmd.fixed);
    }
    return packets::Error::NONE;
}

void SerialLink::readLoop() {
    while (m_running) {
        while (auto frame = m_serial->receive()) {
            // Observe receipt before mutex contention or device operations can delay processing.
            const uint32_t receivedUs = static_cast<uint32_t>(pros::c::micros());
            const uint32_t receivedMs = pros::millis();
            std::lock_guard<pros::Mutex> lock(m_mutex);
            auto error = frame->type == packets::CONFIG ? configFrame(frame->payload, frame->size) : commandFrame(frame->payload, frame->size, receivedUs, receivedMs);
            if (error != packets::Error::NONE) m_error = error;
        }
        pros::delay(1);
    }
}

void SerialLink::writeLoop() {
    uint32_t wake = pros::millis();
    while (m_running) {
        uint8_t period;
        { std::lock_guard<pros::Mutex> lock(m_mutex); period = m_cfg.fixed.statePeriodMs; }
        pros::Task::delay_until(&wake, period);
        for (int i = 0; i < 2 && m_serial->midFrame(); ++i) pros::delay(1);
        std::unique_lock<pros::Mutex> lock(m_mutex);
        // The timeout is reported in every mode but only brakes during autonomous.
        const bool watchdog = m_cfg.fixed.configId && pros::millis() - m_cmdMs > m_cfg.fixed.watchdogMs;
        if (watchdog && pros::competition::is_autonomous() && !m_braked) { m_hw.brakeGroups(); m_braked = true; }
        if (!m_nanoControl && !pros::competition::is_disabled()) { lock.unlock(); flushOneLog(); continue; }
        brain::BrainState state = m_hw.sample();
        if (m_cfg.fixed.configId && (state.sensors.motorGroupCount != m_cfg.fixed.nGroups || state.sensors.trackingWheelCount != m_cfg.fixed.nRotations)) { m_error = packets::Error::GROUP_COUNT; continue; }
        uint8_t payload[packets::MAX_PAYLOAD]{};
        packets::PayloadWriter writer{payload, sizeof(packets::StateFixed)};
        // Deltas are relative to the last acknowledged state, not the previous sample.
        Snapshot snap; snap.seq = m_seq;
        for (size_t i = 0; i < m_cfg.fixed.nGroups; ++i) {
            const auto& s = state.sensors.motorGroups[i]; snap.group[i] = s.positionRad;
            writer.put(packets::GroupState{static_cast<float>(s.positionRad - m_base.group[i]), s.velocity, s.faultFlags});
        }
        for (size_t i = 0; i < m_cfg.fixed.nRotations; ++i) {
            const auto& s = state.sensors.trackingWheels[i]; snap.wheel[i] = s.positionRad;
            writer.put(packets::RotationState{static_cast<float>(s.positionRad - m_base.wheel[i]), s.faultFlags});
        }
        packets::StateFixed f{};
        f.stateSeq = m_seq; f.deltaBaseSeq = m_base.seq; f.lastCmdSeq = m_lastCmd; f.configId = m_cfg.fixed.configId;
        f.lastRequestId = m_lastRequest; f.lastError = static_cast<uint8_t>(m_error);
        f.status = (state.statusFlags & ~(packets::status::WATCHDOG | packets::status::BOOTED | packets::status::CONTROLLER)) | (watchdog ? packets::status::WATCHDOG : 0) | (m_booted ? packets::status::BOOTED : 0) | (state.driver.connected ? packets::status::CONTROLLER : 0);
        f.batteryMv = state.batteryVoltage; std::memcpy(f.ctrlAxes, state.driver.axes.data(), sizeof f.ctrlAxes); f.ctrlButtons = state.driver.buttons;
        f.rxErrors = m_serial->m_crcErrors + m_serial->m_badFrames; f.nGroups = m_cfg.fixed.nGroups; f.nRotations = m_cfg.fixed.nRotations;
        const bool haveCmd = m_haveCmd;
        const uint32_t cmdUs = m_cmdUs;
        const packets::Error reportedError = m_error;
        // Record the snapshot before releasing the lock so an immediate acknowledgment can find it.
        m_history[m_seq % m_history.size()] = snap;
        m_seq = m_seq == 0xffff ? 1 : m_seq + 1;
        // Writes can block; do not hold the shared state lock while transmitting.
        lock.unlock();
        flushOneLog();
        f.tUs = static_cast<uint32_t>(pros::c::micros());
        f.cmdAgeUs = haveCmd ? f.tUs - cmdUs : packets::NO_COMMAND;
        std::memcpy(payload, &f, sizeof f);
        const bool sent = m_serial->send(packets::STATE, writer);
        if (!sent) ++m_txErrors;
        if (sent && reportedError != packets::Error::NONE) {
            std::lock_guard<pros::Mutex> guard(m_mutex);
            // Preserve a different error recorded while the frame was in flight.
            if (m_error == reportedError) m_error = packets::Error::NONE;
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
    if (!m_serial->send(packets::LOG, payload, static_cast<uint16_t>(1 + entry.length))) ++m_txErrors;
}
