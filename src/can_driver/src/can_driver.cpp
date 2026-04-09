#include "can_driver/can_driver.hpp"

#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

CanDriver::CanDriver() : socket_(-1), is_open_(false) {}

CanDriver::~CanDriver()
{
    close();
}

bool CanDriver::init(const std::string& interface)
{
    struct ifreq ifr;
    struct sockaddr_can addr;

    socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0)
    {
        perror("Socket");
        return false;
    }

    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    ioctl(socket_, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Bind");
        return false;
    }

    is_open_ = true;
    return true;
}

bool CanDriver::sendFrame(const struct can_frame& frame)
{
    if (!is_open_) return false;

    int nbytes = write(socket_, &frame, sizeof(frame));
    return nbytes == sizeof(frame);
}

bool CanDriver::receiveFrame(struct can_frame& frame)
{
    if (!is_open_) return false;

    int nbytes = read(socket_, &frame, sizeof(frame));
    return nbytes == sizeof(frame);
}

void CanDriver::close()
{
    if (is_open_)
    {
        ::close(socket_);
        is_open_ = false;
    }
}