#pragma once
#include <chrono>

/* - - - - - - - - - - - - - - [SHUTDOWN] - - - - - - - - - - - - - - */

/**
 * @brief makes SIGINT (Ctrl+C) and SIGTERM request a clean shutdown instead of killing the program
 */
void installShutdownHandlers();

/**
 * @brief whether a shutdown was requested
 * @returns true after SIGINT or SIGTERM
 */
bool shutdownRequested();

/* - - - - - - - - - - - - - - [TIMING] - - - - - - - - - - - - - - */

/**
 * @class Ticker
 * @brief fixed-rate loop timer
 */
class Ticker {
    public:
        /**
         * @param periodSeconds time between ticks in seconds
         */
        explicit Ticker(double periodSeconds);

        /**
         * @brief sleeps until the next tick
         * @note if a step ran long, restarts the schedule instead of running steps back to back
         */
        void wait();

    private:
        std::chrono::steady_clock::duration m_period; /**< time between ticks */
        std::chrono::steady_clock::time_point m_next; /**< when the current tick started */
};
