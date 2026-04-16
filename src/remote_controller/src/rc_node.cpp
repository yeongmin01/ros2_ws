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

        if (key == 'w' && rc_cmd.velocity < 250) rc_cmd.velocity += 1;
        if (key == 's') rc_cmd.velocity = 0;
        if (key == 'x' && rc_cmd.velocity > 127) rc_cmd.velocity -= 1;
        if (key == 'd' && rc_cmd.steer < 249) rc_cmd.steer += 1;
        if (key == 'a' && rc_cmd.steer > 10) rc_cmd.steer -= 1;
        if (key == 'q') 
        {
            if(rc_cmd.steer > 10)
            {
                rc_cmd.steer -= 1;
            }
            if(rc_cmd.velocity < 250)
            {
                rc_cmd.velocity += 1;
            }
        }
        if (key == 'e') 
        {
            if(rc_cmd.steer < 249)
            {
                rc_cmd.steer += 1;
            }
            if(rc_cmd.velocity < 250)
            {
                rc_cmd.velocity += 1;
            }
        }

        if (key == '1') 
        {
            rc_state.rc_forward = true;
            rc_state.rc_backward = false;
        }
        if (key == '2') 
        {
            rc_state.rc_forward = false;
            rc_state.rc_backward = true;
        }
        if (key == '3') 
        {
            rc_state.rc_not_crab = !rc_state.rc_not_crab;
        }

        publishRCCmd();  // 항상 실행
        publishRCState();

        RCLCPP_INFO(this->get_logger(),"RC State [%s]  vel: %d steer: %d",getRCLog(rc_state).c_str(),  rc_cmd.velocity, rc_cmd.steer);
    }
    // RC Velocity, Steering angle Cmd publish
    void publishRCCmd()
    {
        custom_msgs::msg::J1939Msg msg;

        msg.priority = RCProtocol::RCCmdtoVCU_Protocol.priority;
        msg.pgn = RCProtocol::RCCmdtoVCU_Protocol.pgn;
        msg.source_address = RCProtocol::RCCmdtoVCU_Protocol.source_address;
        msg.dest_address = RCProtocol::RCCmdtoVCU_Protocol.dest_address;
        msg.dlc = 8;

        msg.data[0] = rc_cmd.steer & 0xFF;
        msg.data[2] = rc_cmd.velocity & 0xFF;

        pub_->publish(msg);
    }
    // RC state publish
    void publishRCState()
    {
        custom_msgs::msg::J1939Msg msg;

        msg.priority = RCProtocol::RCStatetoVCU_Protocol.priority;
        msg.pgn = RCProtocol::RCStatetoVCU_Protocol.pgn;
        msg.source_address = RCProtocol::RCStatetoVCU_Protocol.source_address;
        msg.dest_address = RCProtocol::RCStatetoVCU_Protocol.dest_address;
        msg.dlc = 8;

        msg.data[2] = (msg.data[2] & ~(1 << 5)) | (rc_state.rc_not_crab << 5);
        msg.data[2] = (msg.data[2] & ~(1 << 6)) | (rc_state.rc_forward << 6);
        msg.data[2] = (msg.data[2] & ~(1 << 7)) | (rc_state.rc_backward << 7);
        msg.data[3] = (msg.data[3] & ~(1 << 6)) | (rc_state.rc_start << 6);

        pub_->publish(msg);
    }

    std::string getRCLog(RCProtocol::RCState rc_state)
    {
        std::string rc_movement_log = "";
        std::string rc_crab_mode_log = "";
        if(rc_state.rc_forward && !rc_state.rc_backward)
        {
            rc_movement_log = "Forward";
        }
        if(!rc_state.rc_forward && rc_state.rc_backward)
        {
            rc_movement_log = "Backward";
        }
        if(!rc_state.rc_forward && !rc_state.rc_backward)
        {
            rc_movement_log = "Neutrality";
        }
        if(rc_state.rc_forward && rc_state.rc_backward)
        {
            rc_movement_log = "Invalid";
        }
        if(rc_state.rc_not_crab)
        {
            rc_crab_mode_log = "General";
        }
        else
        {
            rc_crab_mode_log = "Crab";
        }

        return rc_movement_log + ", " + rc_crab_mode_log;
    }

private:
    rclcpp::Publisher<custom_msgs::msg::J1939Msg>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    RCProtocol::RCCmd rc_cmd;
    RCProtocol::RCState rc_state;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RCNode>());
    rclcpp::shutdown();
    return 0;
}