/**
 * @file logging.cpp
 * @brief Project interface or implementation.
 */

#include "daydream/subsystems/logging.hpp"

#include "daydream/utils/sd_card_logging.hpp"

namespace daydream {

void Logging::initialize() {
	Logger::getInstance().init("/usd/log.txt");
}

void Logging::log(const char* message) {
	Logger::getInstance().log(message);
}

}  // namespace daydream
