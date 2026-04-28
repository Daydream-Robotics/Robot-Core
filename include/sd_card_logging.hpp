#pragma once
#include <cstdio>
#include "pros/rtos.hpp"
#include "pros/misc.hpp"

class Logger {
public:
    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }

    void init(const char* path = "/usd/log.txt") {
        if (m_file) fclose(m_file);
        if (!pros::usd::is_installed()) return; // silently proceed
        m_file = fopen(path, "w");
    }

    void log(const char* message) {
        if (!m_file) return; // silently proceed
        fprintf(m_file, "[%lu ms] %s\n", pros::millis(), message);
        fflush(m_file);
    }

    void close() {
        if (m_file) {
            fclose(m_file);
            m_file = nullptr;
        }
    }

    // prevent copies
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

private:
    Logger() = default;
    ~Logger() { close(); }
    FILE* m_file = nullptr;
};

// Convenience macro
#define LOG(msg) Logger::getInstance().log(msg)