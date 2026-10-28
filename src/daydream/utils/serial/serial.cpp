/**
 * @file serial.cpp
 * @brief Link layer for the brain ⇄ Nano link. One block per platform.
 */

#include "daydream/utils/serial/serial.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <mutex>

#ifdef __v5_brain__
    #include <cstdarg>
    #include <cstdio>
    #include "pros/device.hpp"
    #include "pros/error.h"
#endif

#ifdef __jetson_nano__
    #include <cstdio>
    #include <pthread.h>
    #include <sched.h>
#endif


#ifdef __v5_brain__

/// @brief Rumble patterns for Request::CONTROLLER_RUMBLE, indexed by requestValue. Must match the Nano's table.
static const char* const RUMBLE_PATTERNS[] = {".", "-", "..", "--", ".-.", "-.-"};

/**
 * @brief Converts GroupCfg::gearset to the PROS cartridge.
 *
 * @param gearset packets::Gearset value.
 * @return pros::MotorGears The cartridge.
 */
static pros::MotorGears toGears(uint8_t gearset) {
    switch (static_cast<packets::Gearset>(gearset)) {
        case packets::Gearset::RED:  
            return pros::MotorGears::red;
        case packets::Gearset::BLUE: 
            return pros::MotorGears::blue;
        default:                     
            return pros::MotorGears::green;
    }
}

/**
 * @brief Converts GroupCfg::brakeMode to the PROS brake mode.
 *
 * @param brakeMode packets::BrakeMode value.
 * @return pros::MotorBrake The brake mode.
 */
static pros::MotorBrake toBrake(uint8_t brakeMode) {
    switch (static_cast<packets::BrakeMode>(brakeMode)) {
        case packets::BrakeMode::BRAKE: 
            return pros::MotorBrake::brake;
        case packets::BrakeMode::HOLD:  
            return pros::MotorBrake::hold;
        default:                        
            return pros::MotorBrake::coast;
    }
}

/**
 * @brief Whether sequence number `a` comes after `b`.
 *
 * @param a Sequence number to test.
 * @param b Sequence number to compare against. 0 = the config origin.
 * @return bool true if `a` is newer.
 */
static bool isNewer(uint16_t a, uint16_t b) {
    return b == 0 || static_cast<int16_t>(a - b) > 0;
}

SerialLink::SerialLink(uint8_t port, int32_t baud, std::vector<uint8_t> config, SerialProtocol::Transport transport)
    : m_linkPort(port),
      m_baud(baud),
      m_transport(transport),
      m_controller(pros::E_CONTROLLER_MASTER),
      m_bootConfig(std::move(config)) {}

SerialLink::~SerialLink() {
    // stop the tasks before the members they use go away
    if (m_readerTask) {
        m_readerTask->remove();
    }
    if (m_writerTask) {
        m_writerTask->remove();
    }
}

bool SerialLink::start() {
    // open the link
    bool opened = m_transport == SerialProtocol::Transport::USB ? m_serial.setUpVexSerial()
                                                                : m_serial.setUpVexSerial(m_linkPort, m_baud);
    if (!opened) {
        return false;
    }
    // apply the built-in config before any STATE goes out.
    // if it's invalid, the brain runs unconfigured and the first STATE reports the issue
    {
        std::lock_guard<pros::Mutex> lock(m_mutex);
        const uint8_t* payload = m_bootConfig.data();
        auto size = static_cast<uint16_t>(m_bootConfig.size());
        if (!packets::sizeInRange(packets::CONFIG, size) || !packets::sizeExact(packets::CONFIG, payload, size, 0)) {
            m_lastError = packets::Error::SIZE;
        } 
        else if (packets::Error error = applyConfig(payload, size); error != packets::Error::NONE) {
            m_lastError = error;
        }
    }
    // high priority so the 1 ms reader and the tick writer aren't delayed by user tasks.
    // over USB, PROS's stdin read busy-waits for data, so the reader runs at the lowest priority
    // instead and only uses CPU time nothing else wants
    uint32_t readerPriority = m_transport == SerialProtocol::Transport::USB ? TASK_PRIORITY_MIN + 1
                                                                            : TASK_PRIORITY_MAX - 2;
    m_readerTask = std::make_unique<pros::Task>([this] { readerLoop(); },
                                               readerPriority, TASK_STACK_DEPTH_DEFAULT, "serial reader");
    m_writerTask = std::make_unique<pros::Task>([this] { writerLoop(); },
                                               TASK_PRIORITY_MAX - 2, TASK_STACK_DEPTH_DEFAULT, "serial writer");
    return true;
}

void SerialLink::log(uint8_t level, const char* format, ...) {
    LogEntry entry;
    entry.level = level;
    // format into the entry. vsnprintf cuts it off at MAX_LOG_TEXT
    va_list args;
    va_start(args, format);
    int length = std::vsnprintf(entry.text, sizeof(entry.text), format, args);
    va_end(args);
    // nothing to send
    if (length <= 0) {
        return;
    }
    entry.length = static_cast<uint8_t>(std::min<int>(length, packets::MAX_LOG_TEXT));

    std::lock_guard<pros::Mutex> lock(m_logMutex);
    // queue full so drop the line
    if (m_logCount == LOG_QUEUE_SIZE) {
        return;
    }
    // append at the tail of the ring
    m_logQueue[(m_logHead + m_logCount) % LOG_QUEUE_SIZE] = entry;
    m_logCount++;
}

void SerialLink::readerLoop() {
    while (true) {
        // handle every frame that has fully arrived
        while (std::optional<SerialProtocol::Frame> frame = m_serial.receive()) {
            std::lock_guard<pros::Mutex> lock(m_mutex);
            // SerialProtocol only passes CONFIG and COMMAND through on the brain
            packets::Error error = frame->type == packets::CONFIG
                                       ? applyConfig(frame->payload, frame->size)
                                       : applyCommand(frame->payload, frame->size);
            // keep the error for the next STATE
            if (error != packets::Error::NONE) {
                m_lastError = error;
            }
        }
        pros::delay(1);
    }
}

void SerialLink::writerLoop() {
    uint32_t wakeTime = pros::millis();
    uint8_t buf[packets::MAX_PAYLOAD];

    while (true) {
        // wait for the next tick
        uint8_t period;
        {
            std::lock_guard<pros::Mutex> lock(m_mutex);
            period = m_statePeriodMs;
        }
        pros::Task::delay_until(&wakeTime, period);

        // don't start talking over a frame that's still coming in. wait at most 2 ms
        for (int i = 0; i < 2 && m_serial.midFrame(); i++) {
            pros::delay(1);
        }

        packets::StateFixed fixed{};
        // leave room for the fixed part, it's copied in last once tUs is known
        packets::PayloadWriter writer{buf, sizeof(packets::StateFixed)};
        bool cmdSeen;
        uint32_t cmdTime;
        {
            std::lock_guard<pros::Mutex> lock(m_mutex);
            checkWatchdog();

            // read every channel for this STATE
            Snapshot snap;
            snap.seq = m_stateSeq;
            sampleChannels(snap.ch);

            size_t ch = 0;
            for (Group& group : m_groups) {
                packets::GroupState state{};
                // IME change since the delta base
                state.imeDeltaRad = static_cast<float>(snap.ch[ch] - m_base.ch[ch]);
                ch++;
                // average the speed of the motors
                double sum = 0;
                int count = 0;
                for (double rpm : group.motors->get_actual_velocity_all()) {
                    if (std::isfinite(rpm)) {
                        sum += rpm;
                        count++;
                    }
                }
                state.omegaRadS = count > 0 ? static_cast<float>(sum / count * RPM_TO_RAD_S) : 0.0f;
                // OR every motor's fault bits
                for (uint32_t faults : group.motors->get_faults_all()) {
                    if (faults == static_cast<uint32_t>(PROS_ERR)) {
                        state.flags |= packets::fault::MISSING;
                    } 
                    else {
                        state.flags |= faults & 0x0F;
                    }
                }
                // a motor that was missing at config, or didn't read this tick
                bool anyFailed = std::find(group.lastValid.begin(), group.lastValid.end(), false) != group.lastValid.end();
                if (group.missing || anyFailed) {
                    state.flags |= packets::fault::MISSING;
                }
                writer.put(state);
            }
            for (Wheel& wheel : m_wheels) {
                packets::RotationState state{};
                // rotation change since the delta base
                state.deltaRad = static_cast<float>(snap.ch[ch] - m_base.ch[ch]);
                ch++;
                // the last good reading was held
                state.flags = wheel.readFailed ? packets::fault::MISSING : 0;
                writer.put(state);
            }
            
            // keep the snapshot so a later ack can move the base to it
            m_history[m_stateSeq % HISTORY_SIZE] = snap;

            // fill the fixed part
            fixed.stateSeq = m_stateSeq;
            fixed.deltaBaseSeq = m_base.seq;
            fixed.lastCmdSeq = m_lastCmdSeq;
            fixed.configId = m_configId;
            fixed.lastRequestId = m_lastRequestId;
            fixed.lastError = static_cast<uint8_t>(m_lastError);
            fixed.status = statusBits();
            int32_t battery = pros::battery::get_voltage();
            fixed.batteryMv = battery == PROS_ERR ? 0 : static_cast<uint16_t>(battery);
            readController(fixed);
            fixed.rxErrors = m_serial.crcErrors + m_serial.badFrames;
            fixed.nGroups = static_cast<uint8_t>(m_groups.size());
            fixed.nRotations = static_cast<uint8_t>(m_wheels.size());

            // the error is reported once
            m_lastError = packets::Error::NONE;
            cmdSeen = m_haveCmd;
            cmdTime = m_cmdRxUs;
        }

        // at most one LOG, always before STATE
        flushOneLog();

        // stamp the time last, right before sending, so tUs is as close to the send as possible
        fixed.tUs = static_cast<uint32_t>(pros::c::micros());
        fixed.cmdAgeUs = cmdSeen ? fixed.tUs - cmdTime : packets::NO_COMMAND;
        std::memcpy(buf, &fixed, sizeof(fixed));
        m_serial.send(packets::STATE, writer);

        // 1–65535, then wrap to 1
        m_stateSeq = m_stateSeq == 0xFFFF ? 1 : m_stateSeq + 1;
    }
}

packets::Error SerialLink::applyConfig(const uint8_t* payload, uint16_t size) {
    packets::PayloadReader reader{payload, size};
    packets::ConfigFixed cfg;
    reader.get(cfg);

    // already applied the Nano resends CONFIG until it sees our configId
    if (cfg.configId == m_configId) {
        return packets::Error::NONE;
    }
    // validate everything
    if (cfg.configId == 0 || cfg.statePeriodMs < 5 || cfg.statePeriodMs > 50 ||
        cfg.watchdogMs < 20 || cfg.watchdogMs > 1000 ||
        cfg.nGroups < 1 || cfg.nGroups > packets::MAX_GROUPS || cfg.nRotations > packets::MAX_ROTATIONS) {
        return packets::Error::RANGE;
    }
    // expander slots (8–15) need an expander
    if (cfg.adiExpanderPort == 0 && (cfg.adiOutEnable & 0xFF00) != 0) {
        return packets::Error::CFG_PORT;
    }

    // every smart port may be used once
    bool used[22] = {};
    if (m_transport == SerialProtocol::Transport::RS485 && m_linkPort <= 21) {
        used[m_linkPort] = true;
    }
    auto claim = [&used](int port) {
        if (port < 1 || port > 21) {
            return packets::Error::CFG_PORT;
        }
        if (used[port]) {
            return packets::Error::CFG_DUPLICATE;
        }
        used[port] = true;
        return packets::Error::NONE;
    };
    if (cfg.adiExpanderPort != 0) {
        if (packets::Error e = claim(cfg.adiExpanderPort); e != packets::Error::NONE) {
            return e;
        }
    }

    packets::GroupCfg groupCfgs[packets::MAX_GROUPS];
    int totalMotors = 0;
    for (size_t i = 0; i < cfg.nGroups; i++) {
        packets::GroupCfg& g = groupCfgs[i];
        reader.get(g);
        if (g.nMotors < 1 || g.nMotors > packets::MAX_MOTORS || g.gearset > 2 || g.brakeMode > 2) {
            return packets::Error::RANGE;
        }
        for (size_t m = 0; m < packets::MAX_MOTORS; m++) {
            if (m < g.nMotors) {
                if (packets::Error e = claim(std::abs(g.ports[m])); e != packets::Error::NONE) {
                    return e;
                }
            } 
            else if (g.ports[m] != 0) {
                // unused entries must be 0
                return packets::Error::RANGE;
            }
        }
        totalMotors += g.nMotors;
    }
    // the brain only has 21 smart ports
    if (totalMotors > 21) {
        return packets::Error::CFG_LIMIT;
    }

    packets::RotationCfg rotationCfgs[packets::MAX_ROTATIONS];
    for (size_t i = 0; i < cfg.nRotations; i++) {
        reader.get(rotationCfgs[i]);
        if (packets::Error e = claim(rotationCfgs[i].port); e != packets::Error::NONE) {
            return e;
        }
    }

    // tear down the old devices
    // stop the old groups before releasing them
    for (Group& group : m_groups) {
        group.motors->brake();
    }
    m_groups.clear();
    m_wheels.clear();
    for (auto& out : m_adi) {
        out.reset();
    }

    // ---- build the new devices ----
    for (size_t i = 0; i < cfg.nGroups; i++) {
        const packets::GroupCfg& g = groupCfgs[i];
        Group group;
        std::vector<int8_t> ports(g.ports, g.ports + g.nMotors);
        group.motors = std::make_unique<pros::MotorGroup>(ports, toGears(g.gearset));
        // positions in degrees, converted to radians when sampled
        group.motors->set_encoder_units_all(pros::MotorEncoderUnits::degrees);
        group.motors->set_brake_mode_all(toBrake(g.brakeMode));
        // lastValid starts false so the first sample only seeds lastPos
        group.lastPos.assign(g.nMotors, 0.0);
        group.lastValid.assign(g.nMotors, false);
        // don't reject config if a motor is missing and set a flag
        for (int8_t port : ports) {
            if (pros::Device::get_plugged_type(std::abs(port)) != pros::DeviceType::motor) {
                group.missing = true;
            }
        }
        m_groups.push_back(std::move(group));
    }

    for (size_t i = 0; i < cfg.nRotations; i++) {
        Wheel wheel;
        wheel.sensor = std::make_unique<pros::Rotation>(rotationCfgs[i].port);
        wheel.sensor->set_reversed(rotationCfgs[i].flags & 0x01);
        m_wheels.push_back(std::move(wheel));
    }

    // don't reject config if a expander is missing and set a flag
    m_expanderMissing = cfg.adiExpanderPort != 0 &&
                       pros::Device::get_plugged_type(cfg.adiExpanderPort) != pros::DeviceType::adi;

    for (int k = 0; k < packets::MAX_ADI; k++) {
        if (((cfg.adiOutEnable >> k) & 1) == 0) {
            continue;
        }
        // physical level = logical state XOR active-low
        bool level = ((cfg.adiInitial ^ cfg.adiActiveLow) >> k) & 1;
        if (k < 8) {
            // brain ports A–H
            m_adi[k] = std::make_unique<pros::adi::DigitalOut>(static_cast<uint8_t>('A' + k), level);
        } 
        else {
            // expander ports A–H
            pros::adi::ext_adi_port_pair_t pair{cfg.adiExpanderPort, static_cast<uint8_t>('A' + k - 8)};
            m_adi[k] = std::make_unique<pros::adi::DigitalOut>(pair, level);
        }
    }
    m_adiEnable = cfg.adiOutEnable;
    m_adiActiveLow = cfg.adiActiveLow;

    // reset the deltas to this moment
    // the first sample seeds every motor's lastPos, so the groups start at 0
    m_base = Snapshot{};
    sampleChannels(m_base.ch);
    m_history.fill(Snapshot{});

    m_configId = cfg.configId;
    m_statePeriodMs = cfg.statePeriodMs;
    m_watchdogMs = cfg.watchdogMs;
    m_serial.setCommandGroups(cfg.nGroups);
    // the watchdog holds every group stopped until the first COMMAND
    m_haveCmd = false;
    m_watchdogTripped = false;
    m_watchdogBraked = false;
    m_lastCmdSeq = 0;
    return packets::Error::NONE;
}   

packets::Error SerialLink::applyCommand(const uint8_t* payload, uint16_t size) {
    if (m_configId == 0) {
        return packets::Error::NOT_CONFIGURED;
    }
    packets::PayloadReader reader{payload, size};
    packets::CommandFixed command;
    reader.get(command);
    if (command.configId != m_configId) {
        return packets::Error::CONFIG_ID;
    }
    if (command.nGroups != m_groups.size()) {
        return packets::Error::GROUP_COUNT;
    }

    // check every group command before applying any of them
    packets::GroupCmd cmds[packets::MAX_GROUPS];
    for (size_t i = 0; i < m_groups.size(); i++) {
        reader.get(cmds[i]);
        if (!std::isfinite(cmds[i].value)) {
            return packets::Error::RANGE;
        }
    }

    // valid: feed the watchdog and remember when it arrived for cmdAgeUs
    m_cmdRxUs = static_cast<uint32_t>(pros::c::micros());
    m_cmdRxMs = pros::millis();
    m_haveCmd = true;
    // the Nano has seen this program's STATEs, so it no longer needs the booted flag
    m_booted = false;
    m_lastCmdSeq = command.cmdSeq;

    // move the delta base to the acked STATE, if it's newer and still in the history
    for (const Snapshot& snap : m_history) {
        if (snap.seq != 0 && snap.seq == command.ackStateSeq && isNewer(snap.seq, m_base.seq)) {
            m_base = snap;
            break;
        }
    }

    // apply each group's command
    for (size_t i = 0; i < m_groups.size(); i++) {
        pros::MotorGroup& motors = *m_groups[i].motors;
        const packets::GroupCmd& cmd = cmds[i];
        switch (static_cast<packets::GroupMode>(cmd.mode)) {
            case packets::GroupMode::VOLTAGE:
                // clamp to the motor's range
                motors.move_voltage(std::clamp<int32_t>(std::lround(cmd.value), -12000, 12000));
                break;
            case packets::GroupMode::VELOCITY:
                motors.move_velocity(std::lround(cmd.value));
                break;
            case packets::GroupMode::POSITION:
                motors.move_absolute(cmd.value, cmd.aux);
                break;
            default:
                // STOP, and any unknown mode, stops with the group's brake mode
                motors.brake();
                break;
        }
    }

    // ADI: only slots in the mask that are enabled outputs
    uint16_t mask = command.adiOutMask & m_adiEnable;
    for (int k = 0; k < packets::MAX_ADI; k++) {
        if ((mask >> k) & 1) {
            // physical level = logical state XOR active-low
            m_adi[k]->set_value(((command.adiOutState ^ m_adiActiveLow) >> k) & 1);
        }
    }

    // a new requestId runs its request once
    if (command.requestId != 0 && command.requestId != m_lastRequestId) {
        m_lastRequestId = command.requestId;
        return runRequest(command);
    }
    return packets::Error::NONE;
}

packets::Error SerialLink::runRequest(const packets::CommandFixed& command) {
    switch (static_cast<packets::Request>(command.requestType)) {
        case packets::Request::NONE:
            return packets::Error::NONE;

        case packets::Request::GROUP_TARE: {
            if (command.requestTarget >= m_groups.size()) {
                return packets::Error::REQ_TARGET;
            }
            Group& group = m_groups[command.requestTarget];
            // zero the motors' absolute position for POSITION mode
            if (group.motors->tare_position_all() == PROS_ERR) {
                return packets::Error::REQ_DEVICE;
            }
            // re-seed from the new readings so the tare doesn't show up as motion.
            // the accumulator is untouched, so the reported deltas don't jump
            std::vector<double> pos = group.motors->get_position_all();
            for (size_t i = 0; i < group.lastPos.size() && i < pos.size(); i++) {
                group.lastValid[i] = pos[i] != PROS_ERR_F;
                if (group.lastValid[i]) {
                    group.lastPos[i] = pos[i];
                }
            }
            return packets::Error::NONE;
        }
        case packets::Request::CONTROLLER_RUMBLE: {
            // requestValue is an index into the pattern table
            size_t index = static_cast<size_t>(command.requestValue);
            if (command.requestValue < 0 || index >= std::size(RUMBLE_PATTERNS)) {
                return packets::Error::RANGE;
            }
            // the controller rate-limits rumble (~50 ms), so this can fail and be retried with a new request
            if (m_controller.rumble(RUMBLE_PATTERNS[index]) == PROS_ERR) {
                return packets::Error::REQ_DEVICE;
            }
            return packets::Error::NONE;
        }
        default:
            return packets::Error::REQ_TYPE;
    }
}

void SerialLink::sampleChannels(double* channels) {
    size_t c = 0;
    for (Group& group : m_groups) {
        std::vector<double> pos = group.motors->get_position_all();
        // add the mean increment of the motors that were valid both last time and now
        double sum = 0;
        int count = 0;
        for (size_t i = 0; i < group.lastPos.size() && i < pos.size(); i++) {
            bool valid = pos[i] != PROS_ERR_F;
            if (valid && group.lastValid[i]) {
                sum += (pos[i] - group.lastPos[i]) * DEG_TO_RAD;
                count++;
            }
            // a motor that comes back only re-seeds, so it can't cause a jump
            if (valid) {
                group.lastPos[i] = pos[i];
            }
            group.lastValid[i] = valid;
        }
        if (count > 0) {
            group.accumRad += sum / count;
        }
        channels[c++] = group.accumRad;
    }
    for (Wheel& wheel : m_wheels) {
        // in centidegrees
        int32_t pos = wheel.sensor->get_position();
        wheel.readFailed = pos == PROS_ERR;
        // hold the last good reading on a failed read
        if (!wheel.readFailed) {
            wheel.last = pos * CDEG_TO_RAD;
        }
        channels[c++] = wheel.last;
    }
}

void SerialLink::checkWatchdog() {
    // nothing to stop before a config
    if (m_configId == 0) {
        return;
    }
    m_watchdogTripped = !m_haveCmd || pros::millis() - m_cmdRxMs > m_watchdogMs;
    // a valid COMMAND re-arms the brake for the next trip
    if (!m_watchdogTripped) {
        m_watchdogBraked = false;
        return;
    }
    // only brake in autonomous, once per trip. ADI outputs keep their state in every mode
    if (pros::competition::is_autonomous() && !m_watchdogBraked) {
        for (Group& group : m_groups) {
            group.motors->brake();
        }
        m_watchdogBraked = true;
    }
}

uint8_t SerialLink::statusBits() {
    uint8_t status = 0;
    if (pros::competition::is_disabled()) {
        status |= packets::status::DISABLED;
    }
    if (pros::competition::is_autonomous()) {
        status |= packets::status::AUTON;
    }
    if (pros::competition::is_connected()) {
        status |= packets::status::COMP;
    }
    if (pros::competition::is_field_control()) {
        status |= packets::status::FIELD;
    }
    if (m_watchdogTripped) {
        status |= packets::status::WATCHDOG;
    }
    // is_connected() returns PROS_ERR on failure, so compare against 1
    if (m_controller.is_connected() == 1) {
        status |= packets::status::CONTROLLER;
    }
    if (m_expanderMissing) {
        status |= packets::status::NO_EXPANDER;
    }
    if (m_booted) {
        status |= packets::status::BOOTED;
    }
    return status;
}

void SerialLink::readController(packets::StateFixed& fixed) {
    // PROS orders the axes LX, LY, RX, RY
    for (int i = 0; i < 4; i++) {
        auto axis = static_cast<pros::controller_analog_e_t>(pros::E_CONTROLLER_ANALOG_LEFT_X + i);
        int32_t value = m_controller.get_analog(axis);
        fixed.ctrlAxes[i] = value == PROS_ERR ? 0 : static_cast<int8_t>(std::clamp<int32_t>(value, -127, 127));
    }
    // PROS orders the buttons L1 … A
    fixed.ctrlButtons = 0;
    for (int i = 0; i < 12; i++) {
        auto button = static_cast<pros::controller_digital_e_t>(pros::E_CONTROLLER_DIGITAL_L1 + i);
        // get_digital() returns PROS_ERR on failure, so compare against 1
        if (m_controller.get_digital(button) == 1) {
            fixed.ctrlButtons |= 1 << i;
        }
    }
}

void SerialLink::flushOneLog() {
    LogEntry entry;
    {
        std::lock_guard<pros::Mutex> lock(m_logMutex);
        if (m_logCount == 0) {
            return;
        }
        // pop the oldest line
        entry = m_logQueue[m_logHead];
        m_logHead = (m_logHead + 1) % LOG_QUEUE_SIZE;
        m_logCount--;
    }
    // payload: 1 level byte, then the text without a terminator
    uint8_t payload[1 + packets::MAX_LOG_TEXT];
    payload[0] = entry.level;
    std::memcpy(payload + 1, entry.text, entry.length);
    m_serial.send(packets::LOG, payload, static_cast<uint16_t>(1 + entry.length));
}

#endif


#ifdef __jetson_nano__

/**
 * @brief Current steady_clock time.
 *
 * @return int64_t microseconds since a fixed point.
 */
static int64_t nowUs() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

#endif
