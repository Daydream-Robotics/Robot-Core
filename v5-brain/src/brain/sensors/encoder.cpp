/** @file encoder.cpp
 *  @brief Implementation of the PROS rotation-sensor adapter.
 */
#include "./brain/sensors/encoder.hpp"

#include "pros/rotation.hpp"
#include "pros/error.h"

#include <cstdlib>
#include <cstdint>

    namespace brain {

    Encoder::Encoder(std::int8_t smartPort, bool reversed)
        : m_device(static_cast<std::int8_t>(reversed ? -std::abs(static_cast<int>(smartPort))
                                                    : std::abs(static_cast<int>(smartPort)))) {
    }

    void Encoder::update() {
        if (!connected()) {
            //TODO add true error handling (eg. SD Card Printing)
            m_delta = 0.0;
            return;
        }

        const auto rawPosition = m_device.get_position();
        if (rawPosition == PROS_ERR) {
            //TODO add true error handling (eg. SD Card Printing)
            m_delta = 0.0;
            return;
        }

        const double currentPosition = static_cast<double>(rawPosition);
        m_delta = currentPosition - m_position;
        m_position = currentPosition;
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

    void Encoder::tare() {
        if (connected()) {
            m_device.reset_position();
        }
        //TODO add true error handling (eg. SD Card Printing)
        m_position = 0.0;
        m_delta = 0.0;
    }

}