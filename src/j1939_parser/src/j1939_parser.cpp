#include "j1939_parser/j1939_parser.hpp"

// J1939 Msg Format -> Raw CAN Msg Format
struct can_frame J1939Parser::build(const custom_msgs::msg::J1939Msg &msg)
{
    struct can_frame frame{};
    uint32_t id = 0;

    uint8_t pf = (msg.pgn >> 8) & 0xFF;
    uint8_t ps = msg.pgn & 0xFF;

    id |= (msg.priority & 0x7) << 26;

    if (pf < 240)
        ps = msg.dest_address;

    id |= (pf << 16);
    id |= (ps << 8);
    id |= msg.source_address;

    frame.can_id = id | CAN_EFF_FLAG;
    frame.can_dlc = msg.dlc;

    for (int i = 0; i < msg.dlc; i++)
        frame.data[i] = msg.data[i];

    return frame;
}

// Raw CAN Msg -> J1939 PGN 추출
uint32_t J1939Parser::extractPGN(uint32_t can_id)
{
    uint32_t id = can_id & CAN_EFF_MASK;

    uint8_t dp = (id >> 24) & 0x01;
    uint8_t pf = (id >> 16) & 0xFF;
    uint8_t ps = (id >> 8) & 0xFF;

    if (pf < 240)
    {
        return (dp << 16) | (pf << 8);
    }
    else
    {
        return (dp << 16) | (pf << 8) | ps;
    }
}

// Raw CAN Msg Format -> J1939 Msg Format
custom_msgs::msg::J1939Msg J1939Parser::parse(const struct can_frame& frame)
{
    custom_msgs::msg::J1939Msg j;

    uint32_t id = frame.can_id & CAN_EFF_MASK;

    j.priority = (id >> 26) & 0x07;
    j.source_address = id & 0xFF;
    j.pgn = extractPGN(id);
    j.dlc = frame.can_dlc;

    for (int i = 0; i < frame.can_dlc; i++)
    {
        j.data[i] = frame.data[i];
    }

    return j;
}