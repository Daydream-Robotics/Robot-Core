/**
 * @file logging.hpp
 * @brief Project interface or implementation.
 */

#pragma once

namespace daydream {

/** @brief Provides the application-facing logging subsystem interface. */
class Logging {
public:
	/** @brief Initializes the configured logging backend. */
	void initialize();
	/** @brief Writes a null-terminated message through the configured backend. */
	void log(const char* message);
};

}  // namespace daydream
