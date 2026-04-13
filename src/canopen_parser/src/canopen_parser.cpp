#include "canopen_parser/canopen_parser.hpp"

// CANopen 여부 판단 (Standard Frame)
bool CANopenParser::isCANopen(uint32_t can_id)
{
    return !(can_id & CAN_EFF_FLAG);
}

// COB-ID 추출
uint16_t CANopenParser::extractCobId(uint32_t can_id)
{
    return can_id & CAN_SFF_MASK;
}

// Raw CAN → CANopen
custom_msgs::msg::CanopenMsg CANopenParser::parse(const custom_msgs::msg::CanFrame &frame)
{
    custom_msgs::msg::CanopenMsg msg{};

    uint16_t cob_id = extractCobId(frame.id);

    msg.cob_id = cob_id;
    msg.function_code = (cob_id >> 7) & 0x0F;
    msg.node_id = cob_id & 0x7F;
    msg.dlc = frame.dlc;

    for (int i = 0; i < frame.dlc; i++)
        msg.data[i] = frame.data[i];

    return msg;
}

// CANopen → Raw CAN
custom_msgs::msg::CanFrame CANopenParser::build(const custom_msgs::msg::CanopenMsg &msg)
{
    custom_msgs::msg::CanFrame frame{};

    uint16_t cob_id = ((msg.function_code & 0x0F) << 7) |
                      (msg.node_id & 0x7F);

    frame.id = cob_id; // Standard Frame (EFF 없음)
    frame.dlc = msg.dlc;

    for (int i = 0; i < msg.dlc; i++)
        frame.data[i] = msg.data[i];

    return frame;
}