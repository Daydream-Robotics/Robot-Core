#pragma once

#include "brain/types.hpp"

#include "pros/misc.hpp"

namespace brain {

class DriverInputReader {
public:
    explicit DriverInputReader(pros::Controller& controller);

    DriverInput read() const;

private:
    pros::Controller& controller_;
};

} // namespace brain
