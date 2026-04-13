#pragma once

#include <linux/can.h>
#include "custom_msgs/msg/can_frame.hpp"
#include "custom_msgs/msg/canopen_msg.hpp"

class CANopenParser
{
public:
    static custom_msgs::msg::CanFrame build(const custom_msgs::msg::CanopenMsg &msg);
    static custom_msgs::msg::CanopenMsg parse(const custom_msgs::msg::CanFrame &frame);

    static bool isCANopen(uint32_t can_id);

private:
    static uint16_t extractCobId(uint32_t can_id);
};