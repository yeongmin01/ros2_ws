#pragma once

#include <cstdint>

namespace MDProtocol
{
    struct ProtocolConfig
    {
        uint32_t pgn;
        uint8_t priority;
        uint8_t dest_address;
        uint8_t source_address;
    };

    struct MDCmd
    {
        int16_t torque_req; // vcu -> emulator request torque
        int16_t velocity_req; // emulator -> issac sim request velocity
    };

    struct MDCur
    {
        int16_t velocity_cur; // issac sim -> emulator current velocity
        int16_t rpm_cur; // emulator -> vcu current RPM 
    };

    constexpr ProtocolConfig MD1toVCU_Protocol = {0xF3CB, 6, 0xCB, 0x5B};
    constexpr ProtocolConfig MD2toVCU_Protocol = {0xF3CB, 6, 0xCB, 0x5A};
    constexpr ProtocolConfig VCUtoMD1_Protocol = {0xF3E2, 6, 0xE2, 0x27};
    constexpr ProtocolConfig VCUtoMD2_Protocol = {0xF3E2, 6, 0xE2, 0x26};
}