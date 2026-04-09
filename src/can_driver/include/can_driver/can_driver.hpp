#pragma once

#include <string>
#include <linux/can.h>

class CanDriver
{
public:
    CanDriver();
    ~CanDriver();

    bool init(const std::string& interface);
    bool sendFrame(const struct can_frame& frame);
    bool receiveFrame(struct can_frame& frame);

    void close();

private:
    int socket_;
    bool is_open_;
};