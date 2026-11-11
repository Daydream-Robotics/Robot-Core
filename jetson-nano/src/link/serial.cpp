/** Nano-side serial link. */
#include "protocol/serial.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <mutex>
#include <cstdio>
#include <pthread.h>
#include <sched.h>

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
