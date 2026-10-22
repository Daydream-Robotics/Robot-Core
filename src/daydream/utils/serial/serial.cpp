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
                bool anyFailed = std::find(group.lastOk.begin(), group.lastOk.end(), false) != group.lastOk.end();
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
            
        }

    }
}
#endif
