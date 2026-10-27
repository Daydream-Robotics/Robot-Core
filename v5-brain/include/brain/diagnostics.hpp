#pragma once

#include <cstdint>

namespace brain {

enum class ErrorCode : std::uint8_t {
    LinkUnavailable,
    StaleCommand,
    SensorUnavailable,
    ConfigurationRejected
};

struct ErrorEvent {
    ErrorCode code;
    const char* message;
};

struct LogRecord {
    std::uint32_t timestamp;
    const char* message;
};

class ErrorReporter {
public:
    void report(const ErrorEvent& event);
};

class Logger {
public:
    void log(const LogRecord& record);
    void flush();
};

} // namespace brain
