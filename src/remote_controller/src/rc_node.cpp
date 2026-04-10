#include "rclcpp/rclcpp.hpp"
#include "remote_controller/rc_protocol.hpp"
#include "custom_msgs/msg/rc_cmd.hpp"
#include "custom_msgs/msg/j1939_msg.hpp"

#include <termios.h>
#include <unistd.h>
#include <iostream>

class RCNode : public rclcpp::Node
{
public:
    RCNode() : Node("rc_node")
    {
        pub_ = create_publisher<custom_msgs::msg::J1939Msg>("/j1939_tx", 50);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&RCNode::loop, this));
    }

private:
    int getch_nonblock()
    {
        struct termios oldt, newt;
        int ch = -1;

        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        struct timeval tv = {0, 0};
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);

        if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0)
        {
            ch = getchar();
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return ch;
    }

    void loop()
    {
        int key = getch_nonblock();

        if (key == 'w' && rc_cmd.velocity <= 250) rc_cmd.velocity += 1;
        if (key == 's' && rc_cmd.velocity >= 10) rc_cmd.velocity -= 1;
        if (key == 'a' && rc_cmd.steer < 250) rc_cmd.steer += 1;
        if (key == 'd' && rc_cmd.steer >= 10) rc_cmd.steer -= 1;

        publishJ1939();  // 항상 실행
    }

    void publishJ1939()
    {
        custom_msgs::msg::J1939Msg msg;

        msg.priority = RCProtocol::RCtoVCU_Protocol.priority;
        msg.pgn = RCProtocol::RCtoVCU_Protocol.pgn;
        msg.source_address = RCProtocol::RCtoVCU_Protocol.source_address;
        msg.dest_address = RCProtocol::RCtoVCU_Protocol.dest_address;
        msg.dlc = 8;

        msg.data[0] = rc_cmd.velocity & 0xFF;
        msg.data[2] = rc_cmd.steer & 0xFF;

        pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(),
                    "vel: %d steer: %d",
                    rc_cmd.velocity, rc_cmd.steer);
    }

private:
    rclcpp::Publisher<custom_msgs::msg::J1939Msg>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    RCProtocol::RCCmd rc_cmd;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RCNode>());
    rclcpp::shutdown();
    return 0;
}