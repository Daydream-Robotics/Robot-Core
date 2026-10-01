#pragma once

namespace daydream {

class Subsystem {
public:
    virtual ~Subsystem() = default;

    virtual void initialize() = 0;
    virtual void control() = 0;
};

} // namespace daydream
