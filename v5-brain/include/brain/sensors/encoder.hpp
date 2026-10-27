#pragma once

#include "pros/rotation.hpp"

#include <cstdint>

namespace brain {

class Encoder {
public:
    explicit Encoder(std::int8_t smartPort, bool reversed = false);

    void update();
    double position() const;
    double delta() const;
    bool connected() const;
    void tare();

private:
    pros::Rotation device_;
};

} // namespace brain
