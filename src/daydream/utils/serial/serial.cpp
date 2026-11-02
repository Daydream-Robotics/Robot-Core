/**
 * @file serial.cpp
 * @brief link layer for the brain ⇄ Nano link. one block per platform
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

/// @brief rumble patterns for Request::CONTROLLER_RUMBLE, indexed by requestValue. must match the Nano's table
static const char* const RUMBLE_PATTERNS[] = {".", "-", "..", "--", ".-.", "-.-"};

/**
 * @brief converts GroupCfg::gearset to the PROS cartridge
 *
 * @param gearset packets::Gearset value
 * @return pros::MotorGears the cartridge
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
 * @brief converts GroupCfg::brakeMode to the PROS brake mode
 *
 * @param brakeMode packets::BrakeMode value
 * @return pros::MotorBrake the brake mode
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
 * @brief whether sequence number `a` comes after `b`
 *
 * @param a sequence number to test
 * @param b sequence number to compare against. 0 = the config origin
 * @return bool true if `a` is newer
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
    // apply the built-in config before any state goes out
    {
        std::lock_guard<pros::Mutex> lock(m_mutex);
        const uint8_t* payload = m_bootConfig.data();
        auto size = static_cast<uint16_t>(m_bootConfig.size());
        // check payload for size errors
        if (!packets::sizeInRange(packets::CONFIG, size) || !packets::sizeExact(packets::CONFIG, payload, size, 0)) {
            m_lastError = packets::Error::SIZE;
        } 
        // check payload for applying errors
        else if (packets::Error error = applyConfig(payload, size); error != packets::Error::NONE) {
            m_lastError = error;
        }
    }
    // nano has high priority so the 1 ms reader and the tick writer aren't delayed by user tasks
    // USB has low priority because PROS's stdin read busy-waits for data, at low it only gets CPU
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

void SerialLink::setNanoControl(bool enabled) {
    // switch flag and log to the Nano once, before state/command stops
    if (!enabled && m_nanoControl) {
        log(0, "nano control off: driver control on the brain");
    }
    m_nanoControl = enabled;
}

bool SerialLink::linkQuiet() const {
    // checks if state/commands are not being sent
    return !m_nanoControl && !pros::competition::is_disabled();
}

void SerialLink::readerLoop() {
    while (true) {
        // handle frames that fully arrived
        while (std::optional<SerialProtocol::Frame> frame = m_serial.receive()) {
            std::lock_guard<pros::Mutex> lock(m_mutex);
            // applies config or command based on frame type, or returns error if neither
            packets::Error error = frame->type == packets::CONFIG
                                       ? applyConfig(frame->payload, frame->size)
                                       : applyCommand(frame->payload, frame->size);
            // keep the error for the next state
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

        // don't write over an incoming frame (wait <2ms)
        for (int i = 0; i < 2 && m_serial.midFrame(); i++) {
            pros::delay(1);
        }

        // if driver control send a log but no state
        if (linkQuiet()) {
            flushOneLog();
            continue;
        }

        packets::StateFixed fixed{};
        // prepare the state, it's copied in last once tUs is known
        packets::PayloadWriter writer{buf, sizeof(packets::StateFixed)};
        bool cmdSeen;
        uint32_t cmdTime;
        {
            std::lock_guard<pros::Mutex> lock(m_mutex);
            checkWatchdog();
            checkDevices();

            // read every channel for this state
            Snapshot snap;
            snap.seq = m_stateSeq;
            sampleChannels(snap.ch);

            size_t ch = 0;
            for (Group& group : m_groups) {
                packets::GroupState state{};
                // IME change since the delta base (last acked state)
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
                // flag missing if a motor is unplugged now (checkDevices) or its position didn't read this tick
                bool anyFailed = std::find(group.lastValid.begin(), group.lastValid.end(), false) != group.lastValid.end();
                if (group.missing || anyFailed) {
                    state.flags |= packets::fault::MISSING;
                }
                // write into the state payload
                writer.put(state);
            }
            for (Wheel& wheel : m_wheels) {
                packets::RotationState state{};
                // rotation change since the delta base (last acked state)
                state.deltaRad = static_cast<float>(snap.ch[ch] - m_base.ch[ch]);
                ch++;
                // if the last good reading was held
                state.flags = wheel.readFailed || wheel.missing ? packets::fault::MISSING : 0;
                // write into the state payload
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
            fixed.rxErrors = m_serial.m_crcErrors + m_serial.m_badFrames;
            fixed.nGroups = static_cast<uint8_t>(m_groups.size());
            fixed.nRotations = static_cast<uint8_t>(m_wheels.size());

            // the report error once
            m_lastError = packets::Error::NONE;
            cmdSeen = m_haveCmd;
            cmdTime = m_cmdRxUs;
        }

        // log once
        flushOneLog();

        // stamp the time right before sending
        fixed.tUs = static_cast<uint32_t>(pros::c::micros());
        fixed.cmdAgeUs = cmdSeen ? fixed.tUs - cmdTime : packets::NO_COMMAND;
        // copy to buffer and send
        std::memcpy(buf, &fixed, sizeof(fixed));
        m_serial.send(packets::STATE, writer);

        // 1–65535, then wrap to 1
        m_stateSeq = m_stateSeq == 0xFFFF ? 1 : m_stateSeq + 1;
    }
}

packets::Error SerialLink::applyConfig(const uint8_t* payload, uint16_t size) {
    // fwd cursor
    packets::PayloadReader reader{payload, size};
    packets::ConfigFixed cfg;
    reader.get(cfg);

    // if already applied then do nothing and return
    if (cfg.configId == m_configId) {
        return packets::Error::NONE;
    }
    // range validate everything
    if (cfg.configId == 0 || cfg.statePeriodMs < 5 || cfg.statePeriodMs > 50 ||
        cfg.watchdogMs < 20 || cfg.watchdogMs > 1000 ||
        cfg.nGroups < 1 || cfg.nGroups > packets::MAX_GROUPS || cfg.nRotations > packets::MAX_ROTATIONS) {
        return packets::Error::RANGE;
    }
    // if usee, expander slots (8–15) need an expander
    if (cfg.adiExpanderPort == 0 && (cfg.adiOutEnable & 0xFF00) != 0) {
        return packets::Error::CFG_PORT;
    }

    // check every smart port is used once
    bool used[22] = {};
    if (m_transport == SerialProtocol::Transport::RS485 && m_linkPort <= 21) {
        used[m_linkPort] = true;
    }
    // lambda to claim ports or return error
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
    // check validity of ADI expander port and claim it
    if (cfg.adiExpanderPort != 0) {
        if (packets::Error e = claim(cfg.adiExpanderPort); e != packets::Error::NONE) {
            return e;
        }
    }

    packets::GroupCfg groupCfgs[packets::MAX_GROUPS];
    int totalMotors = 0;
    // check and claim all smart ports for all motor groups
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
                // unused entries are 0
                return packets::Error::RANGE;
            }
        }
        totalMotors += g.nMotors;
    }
    // check number of smart ports used
    if (totalMotors > 21) {
        return packets::Error::CFG_LIMIT;
    }

    // check and claim all smart ports for all rotation sensors
    packets::RotationCfg rotationCfgs[packets::MAX_ROTATIONS];
    for (size_t i = 0; i < cfg.nRotations; i++) {
        reader.get(rotationCfgs[i]);
        if (packets::Error e = claim(rotationCfgs[i].port); e != packets::Error::NONE) {
            return e;
        }
    }

    // mark the motor ports the new groups use
    bool newMotorPort[22] = {};
    for (size_t i = 0; i < cfg.nGroups; i++) {
        for (size_t m = 0; m < groupCfgs[i].nMotors; m++) {
            newMotorPort[std::abs(groupCfgs[i].ports[m])] = true;
        }
    }
    // brake the old motors on ports the new groups don't use
    for (Group& group : m_groups) {
        for (int8_t port : group.motors->get_port_all()) {
            if (!newMotorPort[std::abs(port)]) {
                pros::c::motor_brake(std::abs(port));
            }
        }
    }

    // outputs that already exist on the same pin keep their level,
    // new outputs start at adiInitial, flipped if active-low
    uint16_t keepAdi = m_adiEnable & cfg.adiOutEnable;
    if (cfg.adiExpanderPort != m_expanderPort) {
        // the expander moved, so slots 8–15 are different pins
        keepAdi &= 0x00FF;
    }
    uint16_t adiLevel = (keepAdi & m_adiLevel) | (~keepAdi & (cfg.adiInitial ^ cfg.adiActiveLow) & cfg.adiOutEnable);

    // release old devices
    m_groups.clear();
    m_wheels.clear();
    for (auto& out : m_adi) {
        out.reset();
    }

    // build the new motor groups
    for (size_t i = 0; i < cfg.nGroups; i++) {
        const packets::GroupCfg& g = groupCfgs[i];
        Group group;
        std::vector<int8_t> ports(g.ports, g.ports + g.nMotors);
        group.motors = std::make_unique<pros::MotorGroup>(ports, toGears(g.gearset));
        group.motors->set_encoder_units_all(pros::MotorEncoderUnits::degrees);
        group.motors->set_brake_mode_all(toBrake(g.brakeMode));
        group.gears = toGears(g.gearset);
        group.brake = toBrake(g.brakeMode);
        // lastValid starts false so the first sample just seeds lastPos
        group.lastPos.assign(g.nMotors, 0.0);
        group.lastValid.assign(g.nMotors, false);
        // if a motor is missing set a flag
        for (int8_t port : ports) {
            if (pros::Device::get_plugged_type(std::abs(port)) != pros::DeviceType::motor) {
                group.missing = true;
            }
        }
        m_groups.push_back(std::move(group));
    }

    // build the new rotation sensors
    for (size_t i = 0; i < cfg.nRotations; i++) {
        Wheel wheel;
        wheel.sensor = std::make_unique<pros::Rotation>(rotationCfgs[i].port);
        wheel.reversed = rotationCfgs[i].flags & 0x01;
        wheel.sensor->set_reversed(wheel.reversed);
        // if the sensor is missing set a flag
        wheel.missing = pros::Device::get_plugged_type(rotationCfgs[i].port) != pros::DeviceType::rotation;
        m_wheels.push_back(std::move(wheel));
    }

    // if an expander is missing set a flag
    m_expanderMissing = cfg.adiExpanderPort != 0 && pros::Device::get_plugged_type(cfg.adiExpanderPort) != pros::DeviceType::adi;

    // build ADI devices
    for (int k = 0; k < packets::MAX_ADI; k++) {
        // skip slots that aren't enabled as outputs
        if (((cfg.adiOutEnable >> k) & 1) == 0) {
            continue;
        }
        // set pin level for the slot
        bool level = (adiLevel >> k) & 1;
        if (k < 8) {
            // slots 0–7: output on brain port A–H, set to that level
            m_adi[k] = std::make_unique<pros::adi::DigitalOut>(static_cast<uint8_t>('A' + k), level);
        }
        else {
            // slots 8–15: output on expander port A–H, set to that level
            pros::adi::ext_adi_port_pair_t pair{cfg.adiExpanderPort, static_cast<uint8_t>('A' + k - 8)};
            m_adi[k] = std::make_unique<pros::adi::DigitalOut>(pair, level);
        }
    }
    // store which slots are outputs and which are active-low, used by applyCommand
    m_adiEnable = cfg.adiOutEnable;
    m_adiActiveLow = cfg.adiActiveLow;
    // store the pin level of every output and the expander port, used by checkDevices
    m_adiLevel = adiLevel;
    m_expanderPort = cfg.adiExpanderPort;

    // reset the deltas to this point
    m_base = Snapshot{};
    sampleChannels(m_base.ch);
    m_history.fill(Snapshot{});

    m_configId = cfg.configId;
    m_statePeriodMs = cfg.statePeriodMs;
    m_watchdogMs = cfg.watchdogMs;
    m_serial.setCommandGroups(cfg.nGroups);
    m_cmdRxMs = pros::millis();
    m_haveCmd = false;
    m_watchdogTripped = false;
    m_watchdogBraked = false;
    m_lastCmdSeq = 0;
    return packets::Error::NONE;
}   

packets::Error SerialLink::applyCommand(const uint8_t* payload, uint16_t size) {
    // check if configured
    if (m_configId == 0) {
        return packets::Error::NOT_CONFIGURED;
    }
    // fwd cursor
    packets::PayloadReader reader{payload, size};
    packets::CommandFixed command;
    reader.get(command);
    // check if config matches
    if (command.configId != m_configId) {
        return packets::Error::CONFIG_ID;
    }
    // check motor group size
    if (command.nGroups != m_groups.size()) {
        return packets::Error::GROUP_COUNT;
    }

    // go through all motor group commands
    packets::GroupCmd cmds[packets::MAX_GROUPS];
    for (size_t i = 0; i < m_groups.size(); i++) {
        reader.get(cmds[i]);
        // check every group command before applying 
        if (!std::isfinite(cmds[i].value)) {
            return packets::Error::RANGE;
        }
    }

    // if valid then feed the watchdog and store when it arrived for cmdAgeUs
    m_cmdRxUs = static_cast<uint32_t>(pros::c::micros());
    m_cmdRxMs = pros::millis();
    m_haveCmd = true;
    m_booted = false;
    m_lastCmdSeq = command.cmdSeq;

    // move the delta base to the acked state, if it's newer and still in the history
    for (const Snapshot& snap : m_history) {
        if (snap.seq != 0 && snap.seq == command.ackStateSeq && isNewer(snap.seq, m_base.seq)) {
            m_base = snap;
            break;
        }
    }

    // control motors only in auton
    if (m_nanoControl) {
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
                    motors.brake();
                    break;
            }
        }

        // for ADI only output for devices enabled in the mask
        uint16_t mask = command.adiOutMask & m_adiEnable;
        for (int k = 0; k < packets::MAX_ADI; k++) {
            if ((mask >> k) & 1) {
                // set level from logical state XOR active-low
                bool level = ((command.adiOutState ^ m_adiActiveLow) >> k) & 1;
                m_adi[k]->set_value(level);
                // store level checkDevices() can restore it
                m_adiLevel = level ? m_adiLevel | (1 << k) : m_adiLevel & ~(1 << k);
            }
        }
    }

    // only run a request once
    if (command.requestId != 0 && command.requestId != m_lastRequestId) {
        m_lastRequestId = command.requestId;
        return runRequest(command);
    }
    return packets::Error::NONE;
}

packets::Error SerialLink::runRequest(const packets::CommandFixed& command) {
    // switch for requests
    switch (static_cast<packets::Request>(command.requestType)) {
        case packets::Request::NONE:
            return packets::Error::NONE;

        case packets::Request::GROUP_TARE: {
            // check if it maps to a motor group
            if (command.requestTarget >= m_groups.size()) {
                return packets::Error::REQ_TARGET;
            }
            Group& group = m_groups[command.requestTarget];
            // zero the motors' absolute position for position mode
            if (group.motors->tare_position_all() == PROS_ERR) {
                return packets::Error::REQ_DEVICE;
            }
            // re-seed from the new readings so the tare doesn't show up as motion
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
            // check if requestValue is a number from 0 up to the last pattern in the table
            if (!std::isfinite(command.requestValue) || command.requestValue < 0 ||
                command.requestValue >= static_cast<float>(std::size(RUMBLE_PATTERNS))) {
                return packets::Error::RANGE;
            }
            // convert it to the pattern index
            size_t index = static_cast<size_t>(command.requestValue);
            if (m_controller.rumble(RUMBLE_PATTERNS[index]) == PROS_ERR) {
                return packets::Error::REQ_DEVICE;
            }
            return packets::Error::NONE;
        }
        default:
            // sent wrong request type
            return packets::Error::REQ_TYPE;
    }
}

void SerialLink::sampleChannels(double* channels) {
    // index of the next channel to fill
    size_t c = 0;
    // go through the motor groups
    for (Group& group : m_groups) {
        // read every motor's position in degrees
        std::vector<double> pos = group.motors->get_position_all();
        // total change and number of motors that count this tick
        double sum = 0;
        int count = 0;
        for (size_t i = 0; i < group.lastPos.size() && i < pos.size(); i++) {
            // whether this motor read this tick
            bool valid = pos[i] != PROS_ERR_F;
            // read both last time and now and add its change since last time, in radians
            if (valid && group.lastValid[i]) {
                sum += (pos[i] - group.lastPos[i]) * DEG_TO_RAD;
                count++;
            }
            // read now and store this reading as its new last position
            if (valid) {
                group.lastPos[i] = pos[i];
            }
            // store whether it read, for the next tick
            group.lastValid[i] = valid;
        }
        // add the average change of the motors that counted and if none counted the position holds
        if (count > 0) {
            group.accumRad += sum / count;
        }
        // write the group's position and move to the next channel
        channels[c++] = group.accumRad;
    }
    // go through the tracking wheels
    for (Wheel& wheel : m_wheels) {
        // read the position in centidegrees
        int32_t pos = wheel.sensor->get_position();
        bool valid = pos != PROS_ERR;
        if (valid) {
            // convert to radians
            double rad = pos * CDEG_TO_RAD;
            // if last read was good add the change since then and after a failed read nothing is added
            if (!wheel.readFailed) {
                wheel.accumRad += rad - wheel.last;
            }
            // store this reading as the new last reading
            wheel.last = rad;
        }
        // store whether this read failed
        wheel.readFailed = !valid;
        // write the wheel's position and move to the next channel
        channels[c++] = wheel.accumRad;
    }
}

void SerialLink::checkWatchdog() {
    // nothing to stop before a config
    if (m_configId == 0) {
        return;
    }
    // m_cmdRxMs is the last valid command, or when the config was applied if none since
    m_watchdogTripped = pros::millis() - m_cmdRxMs > m_watchdogMs;
    // a valid command re-arms the brake for the next trip
    if (!m_watchdogTripped) {
        m_watchdogBraked = false;
        return;
    }
    // only brake in autonomous, once per trip
    if (pros::competition::is_autonomous() && !m_watchdogBraked) {
        for (Group& group : m_groups) {
            group.motors->brake();
        }
        m_watchdogBraked = true;
    }
}

void SerialLink::checkDevices() {
    // go through the motor groups
    for (Group& group : m_groups) {
        // missing if any of the group's ports doesn't have a motor plugged in
        bool missing = false;
        for (int8_t port : group.motors->get_port_all()) {
            if (pros::Device::get_plugged_type(std::abs(port)) != pros::DeviceType::motor) {
                missing = true;
            }
        }
        // if every motor is back send the cartridge, encoder units and brake mode again
        if (group.missing && !missing) {
            group.motors->set_gearing_all(group.gears);
            group.motors->set_encoder_units_all(pros::MotorEncoderUnits::degrees);
            group.motors->set_brake_mode_all(group.brake);
        }
        // store the flag for state and the next check
        group.missing = missing;
    }

    // go through the tracking wheels
    for (Wheel& wheel : m_wheels) {
        // missing if the wheel's port doesn't have a rotation sensor plugged in
        bool missing = pros::Device::get_plugged_type(wheel.sensor->get_port()) != pros::DeviceType::rotation;
        // if the sensor is back set its direction again
        if (wheel.missing && !missing) {
            wheel.sensor->set_reversed(wheel.reversed);
        }
        // store the flag for state and the next check
        wheel.missing = missing;
    }

    // if no expander in the config nothing left to check
    if (m_expanderPort == 0) {
        return;
    }
    // missing if the expander's port doesn't have an ADI expander plugged in
    bool missing = pros::Device::get_plugged_type(m_expanderPort) != pros::DeviceType::adi;
    // if the expander is back rebuild each of its outputs at its last level
    if (m_expanderMissing && !missing) {
        for (int k = 8; k < packets::MAX_ADI; k++) {
            // only slots that are outputs
            if (m_adi[k]) {
                // expander port A–H for slot 8–15
                pros::adi::ext_adi_port_pair_t pair{m_expanderPort, static_cast<uint8_t>('A' + k - 8)};
                // replace the output, set to bit k of m_adiLevel
                m_adi[k] = std::make_unique<pros::adi::DigitalOut>(pair, (m_adiLevel >> k) & 1);
            }
        }
    }
    // store the flag for the NO_EXPANDER status bit and the next check
    m_expanderMissing = missing;
}

uint8_t SerialLink::statusBits() {
    // start with every bit clear, then set one bit per true condition
    uint8_t status = 0;
    // robot is disabled
    if (pros::competition::is_disabled()) {
        status |= packets::status::DISABLED;
    }
    // robot is in autonomous
    if (pros::competition::is_autonomous()) {
        status |= packets::status::AUTON;
    }
    // a competition switch or the field is plugged in
    if (pros::competition::is_connected()) {
        status |= packets::status::COMP;
    }
    // field control is plugged in
    if (pros::competition::is_field_control()) {
        status |= packets::status::FIELD;
    }
    // the watchdog is tripped (set by checkWatchdog)
    if (m_watchdogTripped) {
        status |= packets::status::WATCHDOG;
    }
    // the master controller is connected
    if (m_controller.is_connected() == 1) {
        status |= packets::status::CONTROLLER;
    }
    // the configured expander isn't plugged in (set by checkDevices)
    if (m_expanderMissing) {
        status |= packets::status::NO_EXPANDER;
    }
    // no valid command yet since this program started
    if (m_booted) {
        status |= packets::status::BOOTED;
    }
    return status;
}

void SerialLink::readController(packets::StateFixed& fixed) {
    // go through the 4 axes: i = 0 LX, 1 LY, 2 RX, 3 RY
    for (int i = 0; i < 4; i++) {
        // store the PROS axis for index i
        auto axis = static_cast<pros::controller_analog_e_t>(pros::E_CONTROLLER_ANALOG_LEFT_X + i);
        // read the stick, −127..127 or PROS_ERR
        int32_t value = m_controller.get_analog(axis);
        // store it in state (if PROS_ERR stores 0)
        fixed.ctrlAxes[i] = value == PROS_ERR ? 0 : static_cast<int8_t>(std::clamp<int32_t>(value, -127, 127));
    }
    // clear every button bit
    fixed.ctrlButtons = 0;
    // go through the 12 buttons: i = 0 L1, 1 L2, 2 R1, 3 R2, 4 up, 5 down, 6 left, 7 right, 8 X, 9 B, 10 Y, 11 A
    for (int i = 0; i < 12; i++) {
        // store the PROS button for index i
        auto button = static_cast<pros::controller_digital_e_t>(pros::E_CONTROLLER_DIGITAL_L1 + i);
        // if pressed: set bit i (a PROS_ERR means bit is clear)
        if (m_controller.get_digital(button) == 1) {
            fixed.ctrlButtons |= 1 << i;
        }
    }
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
    m_serial.send(packets::LOG, payload, static_cast<uint16_t>(1 + entry.length));
}

#endif


#ifdef __jetson_nano__

/**
 * @brief current steady_clock time
 *
 * @return int64_t microseconds since a fixed point
 */
static int64_t nowUs() {
    // time since steady_clock's fixed start point, converted to whole microseconds
    return std::chrono::duration_cast<std::chrono::microseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}

/**
 * @brief converts a steady_clock time point to microseconds
 *
 * @param t time point
 * @return int64_t microseconds on the same scale as nowUs()
 */
static int64_t toUs(std::chrono::steady_clock::time_point t) {
    // time from steady_clock's start point to t, in whole microseconds
    return std::chrono::duration_cast<std::chrono::microseconds>(t.time_since_epoch()).count();
}

/**
 * @brief converts a numeric baud rate to a termios speed constant
 *
 * @param baud baud rate, e.g. 921600
 * @return speed_t the termios constant, or B0 if the rate isn't supported
 */
static speed_t toSpeed(int32_t baud) {
    // return the termios constant for each supported rate, B0 for any other
    switch (baud) {
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default: return B0;
    }
}

/**
 * @brief time a frame takes on the wire, at 10 bits per byte (8N1)
 *
 * @param bytes bytes on the wire, header and CRC included
 * @param baud baud rate. 0 = no wire to correct for (USB)
 * @return int64_t microseconds
 */
static int64_t airtimeUs(size_t bytes, int32_t baud) {
    // no baud (USB) gives no airtime
    if (baud <= 0) {
        return 0;
    }
    // bits sent (10 per byte) divided by bits per second in microseconds
    return static_cast<int64_t>(bytes) * 10 * 1000000 / baud;
}

/**
 * @brief gives a thread real-time (SCHED_FIFO) priority
 *
 * needs root or CAP_SYS_NICE
 *
 * @param thread thread to change
 * @param priority SCHED_FIFO priority, 1–99
 */
static void makeRealtime(std::thread& thread, int priority) {
    // scheduling settings with only the priority set
    sched_param param{};
    param.sched_priority = priority;
    // switch the thread to SCHED_FIFO at that priority. if it fails the thread keeps normal priority
    (void)pthread_setschedparam(thread.native_handle(), SCHED_FIFO, &param);
}

SerialLink::SerialLink(const char* port, int32_t baud, int deGpio, std::vector<uint8_t> config, SerialProtocol::Transport transport)
    : m_device(port),
      m_baud(baud),
      m_transport(transport),
      m_deGpio(deGpio),
      m_config(std::move(config)) {
    // copy our config's 13-byte fixed part out of the bytes, or leave it all zero if the config is too short
    packets::ConfigFixed fixed{};
    if (m_config.size() >= sizeof(fixed)) {
        std::memcpy(&fixed, m_config.data(), sizeof(fixed));
    }
    // store the config id, group count and state period from it
    m_myConfigId = fixed.configId;
    m_nGroups = fixed.nGroups;
    m_statePeriodMs = fixed.statePeriodMs;

    // fill the mailbox with one stop command (mode 0) per group
    m_groupCmds.assign(m_nGroups, packets::GroupCmd{});
    // set the config id and group count every command carries
    m_commandFixed.configId = m_myConfigId;
    m_commandFixed.nGroups = m_nGroups;
}

SerialLink::~SerialLink() {
    // tell both threads to stop and wake the writer if it's waiting
    m_running = false;
    m_replyCv.notify_all();
    // wait for each thread to finish, if it was started
    if (m_reader.joinable()) {
        m_reader.join();
    }
    if (m_writer.joinable()) {
        m_writer.join();
    }
}

bool SerialLink::start() {
    // termios constant for the baud rate
    speed_t speed = toSpeed(m_baud);
    // fail on an unsupported baud, config id 0, or a config whose size doesn't match its own counts
    auto size = static_cast<uint16_t>(m_config.size());
    if (speed == B0 || m_myConfigId == 0 || !packets::sizeInRange(packets::CONFIG, size) ||
        !packets::sizeExact(packets::CONFIG, m_config.data(), size, 0)) {
        return false;
    }
    // open and set up the UART fail if it can't be opened
    if (!m_serial.setUpMicrocontrollerSerial(m_device.c_str(), speed, m_deGpio)) {
        return false;
    }
    // set the run flag then start the reader and writer threads
    m_running = true;
    m_reader = std::thread([this] { readerLoop(); });
    m_writer = std::thread([this] { writerLoop(); });
    // make both threads real-time with the writer one level above  reader
    makeRealtime(m_reader, 80);
    makeRealtime(m_writer, 81);
    return true;
}

void SerialLink::setGroup(uint8_t slot, packets::GroupMode mode, float value, int16_t aux) {
    // lock the mailbox
    std::lock_guard<std::mutex> lock(m_commandMutex);
    // ignore a slot that isn't a group
    if (slot >= m_groupCmds.size()) {
        return;
    }
    // replace the slot's command with the new mode, aux and value
    m_groupCmds[slot] = packets::GroupCmd{static_cast<uint8_t>(mode), aux, value};
}

void SerialLink::setAdi(uint8_t slot, bool on) {
    // ignore a slot past the last ADI slot
    if (slot >= packets::MAX_ADI) {
        return;
    }
    // bit for this slot
    uint16_t bit = static_cast<uint16_t>(1u << slot);
    // lock the mailbox
    std::lock_guard<std::mutex> lock(m_commandMutex);
    // set the slot's mask bit so every command sends its state
    m_commandFixed.adiOutMask |= bit;
    // set or clear the slot's state bit
    if (on) {
        m_commandFixed.adiOutState |= bit;
    }
    else {
        m_commandFixed.adiOutState &= static_cast<uint16_t>(~bit);
    }
}

uint8_t SerialLink::request(packets::Request type, uint8_t target, float value) {
    // lock the mailbox
    std::lock_guard<std::mutex> lock(m_commandMutex);
    // step to the next id in 1–255
    do {
        m_nextRequestId = m_nextRequestId == 255 ? 1 : m_nextRequestId + 1;
    } while (m_nextRequestId == m_brainRequestId);
    // store the request in the mailbox, carried by every command until the brain echoes the id
    m_commandFixed.requestId = m_nextRequestId;
    m_commandFixed.requestType = static_cast<uint8_t>(type);
    m_commandFixed.requestTarget = target;
    m_commandFixed.requestValue = value;
    // return the id for requestDone()
    return m_nextRequestId;
}

bool SerialLink::requestDone(uint8_t id) const {
    // true if the id isn't 0 and the latest state echoed it
    return id != 0 && m_brainRequestId == id;
}

void SerialLink::onSample(std::function<void(const Sample&)> callback) {
    // lock the callback list and add the callback to it
    std::lock_guard<std::mutex> lock(m_sampleMutex);
    m_callbacks.push_back(std::move(callback));
}

std::optional<SerialLink::Sample> SerialLink::latestSample() const {
    // lock and return a copy of the latest Sample, or nullopt if there's none yet
    std::lock_guard<std::mutex> lock(m_sampleMutex);
    return m_latest;
}

bool SerialLink::linkUp() const {
    // when the latest state arrived
    int64_t last = m_lastStateUs;
    // never heard from the brain
    if (last == 0) {
        return false;
    }
    // true if that was less than LINK_DOWN_PERIODS state periods ago
    return nowUs() - last < static_cast<int64_t>(LINK_DOWN_PERIODS) * m_statePeriodMs * 1000;
}

void SerialLink::readerLoop() {
    // run until the destructor clears m_running
    while (m_running) {
        // wait up to 100 ms for the next valid frame
        std::optional<SerialProtocol::Frame> frame = m_serial.receive();
        // nothing arrived, check m_running and wait again
        if (!frame) {
            continue;
        }
        if (frame->type == packets::STATE) {
            // stamp when the state finished arriving
            Clock::time_point end = Clock::now();
            {
                // lock the handoff and mark a state as waiting for a reply with its end time
                std::lock_guard<std::mutex> lock(m_replyMutex);
                m_stateEnd = end;
                m_statePending = true;
            }
            // wake the writer, then decode the state
            m_replyCv.notify_one();
            handleState(*frame, end);
        }
        else if (frame->type == packets::LOG) {
            // print the brain's log line
            handleLog(*frame);
        }
    }
}

void SerialLink::writerLoop() {
    // start the config counter edge so first mismatch sends config immediatly
    int ticksSinceConfig = CONFIG_RETRY_TICKS;
    // buffer for the command payload
    uint8_t buf[packets::MAX_PAYLOAD];

    // run until the destructor clears m_running
    while (m_running) {
        // wait for a state
        Clock::time_point end;
        {
            // lock the handoff and sleep until a state is pending, the link stops, or 100 ms pass
            std::unique_lock<std::mutex> lock(m_replyMutex);
            m_replyCv.wait_for(lock, std::chrono::milliseconds(100), [this] { return m_statePending || !m_running; });
            // no state (timeout or stopping), go back to the loop check
            if (!m_statePending) {
                continue;
            }
            // take the state and copy its end time
            m_statePending = false;
            end = m_stateEnd;
        }
        // sleep until GUARD_US after the state ended
        std::this_thread::sleep_until(end + std::chrono::microseconds(packets::GUARD_US));
        // if already past REPLY_START_MAX_US after the state, count it and skip this tick and send nothing
        if (Clock::now() > end + std::chrono::microseconds(packets::REPLY_START_MAX_US)) {
            skippedReplies++;
            continue;
        }

        // if the brain has a different config send config at most every CONFIG_RETRY_TICKS ticks and no command
        if (m_brainConfigId != m_myConfigId) {
            if (++ticksSinceConfig >= CONFIG_RETRY_TICKS) {
                m_serial.send(packets::CONFIG, m_config.data(), static_cast<uint16_t>(m_config.size()));
                ticksSinceConfig = 0;
            }
            continue;
        }

        // when configs match reset the counter
        ticksSinceConfig = CONFIG_RETRY_TICKS;

        // build the command payload in buf
        packets::PayloadWriter out{buf};
        {
            // lock the mailbox and copy the fixed part from it
            std::lock_guard<std::mutex> lock(m_commandMutex);
            packets::CommandFixed fixed = m_commandFixed;
            // add the next cmdSeq and the newest recorded state as the ack
            fixed.cmdSeq = ++m_cmdSeq;
            fixed.ackStateSeq = m_ackStateSeq;
            // write the fixed part, then every group's command
            out.put(fixed);
            for (const packets::GroupCmd& cmd : m_groupCmds) {
                out.put(cmd);
            }
        }
        // store when the command starts going out (t0 for time sync), by cmdSeq
        m_cmdSentUs[m_cmdSeq % CMD_TIMES_SIZE] = nowUs();
        // send the command
        m_serial.send(packets::COMMAND, out);
    }
}

void SerialLink::handleState(const SerialProtocol::Frame& frame, Clock::time_point end) {
    // read the 30-byte fixed part
    packets::PayloadReader reader{frame.payload, frame.size};
    packets::StateFixed fixed;
    reader.get(fixed);
    // store arrival time in µs for linkUp()
    int64_t t3 = toUs(end);
    m_lastStateUs = t3;

    // booted (just turned on) so reset the time sync, pending request, and positions
    bool booted = (fixed.status & packets::status::BOOTED) != 0;
    if (booted && !m_brainBooted) {
        resetBrainSession();
        resetPositions();
    }
    // store booted for the next state
    m_brainBooted = booted;
    // if the brain's config id changed reset the positions
    if (fixed.configId != m_brainConfigId) {
        resetPositions();
    }
    // store the brain's config id and last request id
    m_brainConfigId = fixed.configId;
    m_brainRequestId = fixed.lastRequestId;

    {
        // lock the mailbox and clear the request if the brain echoed its id
        std::lock_guard<std::mutex> lock(m_commandMutex);
        if (m_commandFixed.requestId != 0 && m_commandFixed.requestId == fixed.lastRequestId) {
            m_commandFixed.requestId = 0;
            m_commandFixed.requestType = static_cast<uint8_t>(packets::Request::NONE);
        }
    }

    // update the time sync and get the sample time on the Nano clock
    int64_t tNano = syncTime(fixed, frame.size, t3);

    // if the state isn't for our config, return with no Sample and no ack
    if (fixed.configId != m_myConfigId || fixed.nGroups != m_nGroups) {
        return;
    }

    // copy the fixed fields into a new Sample
    Sample sample;
    sample.tNanoUs = tNano;
    sample.stateSeq = fixed.stateSeq;
    sample.status = fixed.status;
    sample.lastRequestId = fixed.lastRequestId;
    sample.lastError = fixed.lastError;
    sample.batteryMv = fixed.batteryMv;
    std::memcpy(sample.ctrlAxes, fixed.ctrlAxes, sizeof(sample.ctrlAxes));
    sample.ctrlButtons = fixed.ctrlButtons;
    sample.rxErrors = fixed.rxErrors;

    // one delta per channel, groups first, then rotations
    size_t channels = fixed.nGroups + fixed.nRotations;
    std::vector<double> delta(channels);
    // read each group's entry, delta into the list, speed and flags into the Sample
    for (size_t i = 0; i < fixed.nGroups; i++) {
        packets::GroupState state;
        reader.get(state);
        delta[i] = state.imeDeltaRad;
        sample.omegaRadS.push_back(state.omegaRadS);
        sample.groupFlags.push_back(state.flags);
    }
    // read each wheel's entry, delta into the list after the groups, flags into the Sample
    for (size_t i = 0; i < fixed.nRotations; i++) {
        packets::RotationState state;
        reader.get(state);
        delta[fixed.nGroups + i] = state.deltaRad;
        sample.wheelFlags.push_back(state.flags);
    }

    // find the base positions (base seq 0 = all zeros otherwise the recorded state with that seq)
    std::vector<double> zeros(channels, 0.0);
    const std::vector<double>* base = nullptr;
    if (fixed.deltaBaseSeq == 0) {
        base = &zeros;
    }
    else {
        // search the history for that seq with the same channel count if not found leaves base null
        for (const PosEntry& entry : m_posHistory) {
            if (entry.seq == fixed.deltaBaseSeq && entry.pos.size() == channels) {
                base = &entry.pos;
                break;
            }
        }
    }

    // previous Sample's positions (or zeros if there's none with this channel count)
    const std::vector<double>& prev = m_lastPos.size() == channels ? m_lastPos : zeros;

    // pos[stateSeq] = pos[base] + delta
    std::vector<double> now(channels);
    if (base) {
        // base found, each position is base plus delta
        for (size_t i = 0; i < channels; i++) {
            now[i] = (*base)[i] + delta[i];
        }
    }
    else {
        // base not found, keep the previous positions and increment counter
        now = prev;
        unknownBases++;
    }

    // add each position and its step since the previous Sample, groups to ime, rotations to wheel
    for (size_t i = 0; i < channels; i++) {
        double step = now[i] - prev[i];
        if (i < fixed.nGroups) {
            sample.imeRad.push_back(now[i]);
            sample.imeStepRad.push_back(step);
        }
        else {
            sample.wheelRad.push_back(now[i]);
            sample.wheelStepRad.push_back(step);
        }
    }

    // record this state's positions, then ack it
    m_posHistory[fixed.stateSeq % POS_HISTORY_SIZE] = PosEntry{fixed.stateSeq, now};
    // keep the positions as the previous ones for the next Sample
    m_lastPos = std::move(now);
    // the writer sends this as ackStateSeq in the next command
    m_ackStateSeq = fixed.stateSeq;

    // copy of the callback list, filled under the lock
    std::vector<std::function<void(const Sample&)>> toCall;
    {
        // lock, store the Sample as the latest and copy the callbacks
        std::lock_guard<std::mutex> lock(m_sampleMutex);
        m_latest = sample;
        toCall = m_callbacks;
    }
    // call each callback with the Sample, with the lock released
    for (auto& callback : toCall) {
        callback(sample);
    }
}

void SerialLink::handleLog(const SerialProtocol::Frame& frame) {
    // print "[brain L<level>] <text>" to stderr. byte 0 is the level, the rest is the text (size - 1 bytes, no terminator)
    std::fprintf(stderr, "[brain L%u] %.*s\n", frame.payload[0], static_cast<int>(frame.size - 1),
                 reinterpret_cast<const char*>(frame.payload + 1));
}

int64_t SerialLink::syncTime(const packets::StateFixed& fixed, uint16_t stateBytes, int64_t t3) {
    // if tUs went down since the last state, (the brain's 32-bit µs clock wrapped) add 2^32 to the high part
    if (m_haveBrainTime && fixed.tUs < m_lastBrainTUs) {
        m_brainTimeHigh += int64_t(1) << 32;
    }
    // store tUs for the next wrap check
    m_lastBrainTUs = fixed.tUs;
    m_haveBrainTime = true;

    // store t2 which is when the brain sent this state, brain clock, unwrapped to 64 bits
    int64_t t2 = m_brainTimeHigh + fixed.tUs;
    // store baud used for airtime, 0 over USB so no airtime is taken off
    int32_t wireBaud = m_transport == SerialProtocol::Transport::USB ? 0 : m_baud;
    // store when the state started arriving = end time minus the frame's airtime
    int64_t t3Start = t3 - airtimeUs(SerialProtocol::HEADER_SIZE + stateBytes + SerialProtocol::CRC_SIZE, wireBaud);

    // if the brain reports a command that isn't used yet, take a time-sync sample from it
    if (fixed.cmdAgeUs != packets::NO_COMMAND && fixed.lastCmdSeq != m_lastSyncedCmd) {
        // mark this command as used
        m_lastSyncedCmd = fixed.lastCmdSeq;
        // store t0, when we started sending that command (Nano clock)
        int64_t t0 = m_cmdSentUs[fixed.lastCmdSeq % CMD_TIMES_SIZE];
        // store t1, when the brain started receiving it (brain clock), t2 minus cmdAgeUs minus the command's airtime
        size_t commandBytes = SerialProtocol::HEADER_SIZE + packets::commandSize(m_nGroups) + SerialProtocol::CRC_SIZE;
        int64_t t1 = t2 - fixed.cmdAgeUs - airtimeUs(commandBytes, wireBaud);
        // store round trip minus the time the brain held the command
        int64_t rtt = (t3Start - t0) - (t2 - t1);
        // store brain clock minus Nano clock
        int64_t offset = ((t1 - t0) + (t2 - t3Start)) / 2;
        // keep the sample only with a send time and a round trip of 0–50 ms
        if (t0 != 0 && rtt >= 0 && rtt < 50000) {
            m_syncWindow.push_back(SyncSample{t3, rtt, offset});
        }
    }

    // drop samples older than SYNC_WINDOW_US from the front
    while (!m_syncWindow.empty() && t3 - m_syncWindow.front().t3 > SYNC_WINDOW_US) {
        m_syncWindow.pop_front();
    }
    // use the offset of the sample with the lowest round trip
    if (!m_syncWindow.empty()) {
        auto best = std::min_element(m_syncWindow.begin(), m_syncWindow.end(),
                                     [](const SyncSample& a, const SyncSample& b) { return a.rtt < b.rtt; });
        m_bestOffset = best->offset;
        m_haveOffset = true;
    }
    // if no offset yet, return when the state started arriving
    if (!m_haveOffset) {
        return t3Start;
    }
    // return the brain's send time converted to the Nano clock
    return t2 - m_bestOffset;
}

void SerialLink::resetBrainSession() {
    // clear the time-sync samples, offset and clock unwrapping
    m_syncWindow.clear();
    m_haveOffset = false;
    m_haveBrainTime = false;
    m_brainTimeHigh = 0;
    m_lastSyncedCmd = 0;
    // lock the mailbox and clear any pending request
    std::lock_guard<std::mutex> lock(m_commandMutex);
    m_commandFixed.requestId = 0;
    m_commandFixed.requestType = static_cast<uint8_t>(packets::Request::NONE);
}

void SerialLink::resetPositions() {
    // empty the position history and previous positions, and clear the ack
    m_posHistory.fill(PosEntry{});
    m_lastPos.clear();
    m_ackStateSeq = 0;
}

#endif
