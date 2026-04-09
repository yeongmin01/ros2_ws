#pragma once

#include <linux/can.h>
#include "custom_msgs/msg/j1939_msg.hpp"
#include "custom_msgs/msg/can_frame.hpp"

class J1939Parser
{
    public:
        static custom_msgs::msg::CanFrame build(const custom_msgs::msg::J1939Msg &msg);
        static custom_msgs::msg::J1939Msg parse(const custom_msgs::msg::CanFrame &frame);
    private:
        static uint32_t extractPGN(uint32_t can_id);
};