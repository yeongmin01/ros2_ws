#pragma once

#include <linux/can.h>
#include "custom_msgs/msg/j1939_msg.hpp"

class J1939Parser
{
    public:
        static struct can_frame build(const custom_msgs::msg::J1939Msg &msg);
        static custom_msgs::msg::J1939Msg parse(const struct can_frame &frame);
    private:
        static uint32_t extractPGN(uint32_t can_id);
};