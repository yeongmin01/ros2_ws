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
        bool gear_r; // vcu -> emulator control gear type
        bool gear_d; // vcu -> emulator control gear type
        bool gear_n; // vcu -> emulator control gear type
        int16_t torque_req; // vcu -> emulator request torque
        float velocity_req; // emulator -> issac sim request velocity
    };

    struct MDCur
    {
        float velocity_cur; // issac sim -> emulator current velocity
        int16_t rpm_cur; // emulator -> vcu current RPM 
    };

    constexpr ProtocolConfig MD1toVCU_Protocol11 = {0xF3CB, 6, 0xCB, 0x5B};
    constexpr ProtocolConfig MD1toVCU_Protocol4 = {0xF3C4, 6, 0xCB, 0x5B};
    constexpr ProtocolConfig MD2toVCU_Protocol11 = {0xF3CB, 6, 0xCB, 0x5A};
    constexpr ProtocolConfig MD2toVCU_Protocol4 = {0xF3C4, 6, 0xCB, 0x5A};
    constexpr ProtocolConfig VCUtoMD1_Protocol1 = {0xF3E1, 6, 0xE1, 0x27};
    constexpr ProtocolConfig VCUtoMD1_Protocol2 = {0xF3E2, 6, 0xE2, 0x27};
    constexpr ProtocolConfig VCUtoMD2_Protocol1 = {0xF3E1, 6, 0xE1, 0x26};
    constexpr ProtocolConfig VCUtoMD2_Protocol2 = {0xF3E2, 6, 0xE2, 0x26};
}