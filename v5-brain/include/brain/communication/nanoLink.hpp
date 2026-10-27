#pragma once

#include "brain/configuration.hpp"
#include "brain/types.hpp"
#include "serial/serialProtocol.hpp"

namespace brain {

struct LinkExchange {
    bool commandReceived = false;
    NanoCommand command{};
    packets::Error error = packets::Error::NONE;
};

class NanoLink {
public:
    explicit NanoLink(SerialProtocol& serial);

    bool configure(const BrainConfiguration& config);
    LinkExchange exchange(const BrainState& state);

private:
    SerialProtocol& serial_;
};

} // namespace brain
