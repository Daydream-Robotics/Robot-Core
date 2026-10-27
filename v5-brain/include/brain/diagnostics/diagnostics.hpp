/** @file diagnostics.hpp
 *  @brief Small value types and interfaces for runtime errors and logging.
 */

#pragma once

#include <cstdint>

namespace brain {

/** @brief Category identifying a reportable Brain runtime error. */
enum class ErrorCode : std::uint8_t {
    /// The Brain cannot exchange data with the Nano.
    LinkUnavailable,
    /// A received Nano command is older than the control path permits.
    StaleCommand,
    /// A configured sensor could not provide usable data.
    SensorUnavailable,
    /// The Nano rejected or could not apply the sent configuration.
    ConfigurationRejected
};

/** @brief Error category and explanatory text suitable for diagnostics. */
struct ErrorEvent {
    /// Machine-readable category of the error.
    ErrorCode code;
    /// Human-readable description of the event.
    const char* message;
};

/** @brief Timestamped text record passed to the Brain logger. */
struct LogRecord {
    /// Event timestamp in the timebase selected by the caller.
    std::uint32_t timestamp;
    /// Text to record.
    const char* message;
};

/** @brief Reports structured runtime errors. */
class ErrorReporter {
public:
    /** @brief Report one runtime error event.
     *  @param event Error code and explanatory message to report.
     */
    void report(const ErrorEvent& event);
};

/** @brief Records and flushes diagnostic log entries. */
class Logger {
public:
    /** @brief Add one record to the diagnostic log.
     *  @param record Timestamped message to record.
     */
    void log(const LogRecord& record);
    /** @brief Flush pending log records to their configured destination. */
    void flush();
};

} // namespace brain
