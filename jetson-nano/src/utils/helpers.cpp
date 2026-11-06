#include "utils/helpers.hpp"
#include <atomic>
#include <csignal>
#include <thread>

/* - - - - - - - - - - - - - - [SHUTDOWN] - - - - - - - - - - - - - - */

static std::atomic<bool> g_shutdown{false};

static void onSignal(int) {
    g_shutdown = true;
}

void installShutdownHandlers() {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
}

bool shutdownRequested() {
    return g_shutdown;
}

/* - - - - - - - - - - - - - - [TIMING] - - - - - - - - - - - - - - */

Ticker::Ticker(double periodSeconds)
    : m_period(std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(periodSeconds))),
      m_next(std::chrono::steady_clock::now()) {}

void Ticker::wait() {
    m_next += m_period;
    const auto now = std::chrono::steady_clock::now();
    if (now > m_next + m_period) {
        m_next = now;
    }
    std::this_thread::sleep_until(m_next);
}
