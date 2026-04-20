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
        uint16_t velocity = 127;
        uint16_t steer = 127;
    };

    struct RCState
    {
        bool rc_start = true;
        bool rc_auto = true;
        bool rc_forward = true;
        bool rc_backward = false;
        bool rc_not_crab = true;
    };

    constexpr ProtocolConfig RCCmdtoVCU_Protocol = {0xF500, 5, 0, 0xC8};
    constexpr ProtocolConfig RCStatetoVCU_Protocol = {0xF501, 5, 0, 0xC8};
}