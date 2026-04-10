// include/your_pkg/j1939_defs.hpp
#pragma once

#include <cstdint>

namespace RCProtocol
{
    struct ProtocolConfig
    {
        uint32_t pgn;
        uint8_t priority;
        uint8_t dest_address;
        uint8_t source_address;
    };

    struct RCCmd
    {
        uint16_t velocity = 0;
        uint16_t steer = 0;
    };

    constexpr ProtocolConfig RCtoVCU_Protocol = {0xF500, 5, 0, 0xC8};
}