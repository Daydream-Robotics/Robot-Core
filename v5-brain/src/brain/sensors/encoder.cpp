/** @file encoder.cpp
 *  @brief Implementation of the PROS rotation-sensor adapter.
 */
#include "brain/sensors/encoder.hpp"

#include <cstdint>
#include <cstdlib>

#include "pros/error.h"
#include "pros/rotation.hpp"

namespace brain {

Encoder::Encoder(std::int8_t smartPort, bool reversed)
    : m_device(static_cast<std::int8_t>(reversed ? -std::abs(static_cast<int>(smartPort))
                                                : std::abs(static_cast<int>(smartPort)))) {
}

void Encoder::update() {
    m_deltaValid = false;
    if (!connected()) {
        //TODO add true error handling (eg. SD Card Printing)
        m_delta = 0.0;
        m_hasBaseline = false;
        return;
    }

    const auto RAW_POSITION = m_device.get_position();
    if (RAW_POSITION == PROS_ERR) {
        //TODO add true error handling (eg. SD Card Printing)
        m_delta = 0.0;
        m_hasBaseline = false;
        return;
    }

    const double CURRENT_POSITION = static_cast<double>(RAW_POSITION);
    m_delta = m_hasBaseline ? CURRENT_POSITION - m_position : 0.0;
    m_deltaValid = m_hasBaseline;
    m_hasBaseline = true;
    m_position = CURRENT_POSITION;
}

double Encoder::getPosition() const {
    return m_position;
}

double Encoder::getDelta() const {
    return m_delta;
}

bool Encoder::connected() const {
    return m_device.get_plugged_type() == pros::DeviceType::rotation;
}

bool Encoder::deltaValid() const { return m_deltaValid; }

void Encoder::tare() {
    if (connected()) {
        m_device.reset_position();
    }
    //TODO add true error handling (eg. SD Card Printing)
    m_position = 0.0;
    m_delta = 0.0;
    m_hasBaseline = false;
    m_deltaValid = false;
}

} // namespace brain
