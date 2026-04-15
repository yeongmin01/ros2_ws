#include "j1939_parser/j1939_parser.hpp"
#include "custom_msgs/msg/can_frame.hpp"
#include "custom_msgs/msg/j1939_msg.hpp"

// J1939 여부 판단
bool J1939Parser::isJ1939(const custom_msgs::msg::CanFrame &frame)
{
    return frame.is_extended;
}

// J1939 Msg Format -> Raw CAN Msg Format
custom_msgs::msg::CanFrame J1939Parser::build(const custom_msgs::msg::J1939Msg &msg)
{
    custom_msgs::msg::CanFrame frame{};
    uint32_t id = 0;

    uint8_t pf = (msg.pgn >> 8) & 0xFF;

    uint8_t ps = 0;

    id |= ((uint32_t)(msg.priority & 0x7)) << 26;

    id |= ((uint32_t)pf) << 16;

    if (pf < 0xF0)
    {
        ps = msg.dest_address;
        id |= ((uint32_t)ps) << 8;
    }
    else
    {
        // broadcast PGN → PS 사용 안 함
        id |= ((msg.pgn & 0xFF)) << 8;
    }

    id |= msg.source_address;

    frame.id = id;  
    frame.dlc = msg.dlc;
    frame.is_extended = true;

    for(int i = 0; i<msg.dlc; i++)
    {
        frame.data[i] = msg.data[i];
    }

    return frame;
}

// Raw CAN Msg -> J1939 PGN 추출
uint32_t J1939Parser::extractPGN(uint32_t can_id)
{
    uint32_t id = can_id;

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
custom_msgs::msg::J1939Msg J1939Parser::parse(const custom_msgs::msg::CanFrame &frame)
{
    custom_msgs::msg::J1939Msg j;

    uint32_t id = frame.id;

    j.priority = (id >> 26) & 0x07;
    j.source_address = id & 0xFF;
    j.pgn = extractPGN(id);
    uint8_t pf = (id >> 16) & 0xFF;
    uint8_t ps = (id >> 8) & 0xFF;

    if (pf < 0xF0)
    {
        j.dest_address = ps;
    }
    else
    {
        j.dest_address = 0xFF; // broadcast
    }
    j.dlc = frame.dlc;

    for (int i = 0; i < frame.dlc; i++)
    {
        j.data[i] = frame.data[i];
    }

    return j;
}