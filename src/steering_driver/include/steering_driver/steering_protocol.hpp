#pragma once

#include <cstdint>
#include <array>

namespace SteeringProtocol
{
    struct ProtocolConfig
    {
        uint8_t function_code;
        uint8_t node_id;
    };

    struct SteeringCmd
    {
        std::array<uint16_t, 4> ampere_req; // VCU -> Emulator FIO ampere
        std::array<float, 4> steering_req; // Emulator -> Issac Sim Steering Cmd (degree)
    };

    struct SteeringCur
    {
        //std::array<float, 4> axis_steering_cur; // Issac Sim -> Emulator Current axis Steering Angle (degree)
        std::array<float, 8> wheel_steering_cur; // Issac Sim -> Emulator Current wheel Steering Angle (degree)
        std::array<uint32_t, 8> postion_cur; // Emulator -> VCU AbsENC Position
    };

    constexpr ProtocolConfig VCUtoFIO_Protocol = {0x6, 0xC};
    constexpr std::array<ProtocolConfig, 8> AbsENC_RtoVCU_Protocol = {ProtocolConfig{0x3, 0x14},
                                                                      ProtocolConfig{0x3, 0x15},
                                                                      ProtocolConfig{0x3, 0x16},
                                                                      ProtocolConfig{0x3, 0x17},
                                                                      ProtocolConfig{0x3, 0x18},
                                                                      ProtocolConfig{0x3, 0x19},
                                                                      ProtocolConfig{0x3, 0x1A},
                                                                      ProtocolConfig{0x3, 0x1B}};
}