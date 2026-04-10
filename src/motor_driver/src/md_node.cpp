#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/j1939_msg.hpp"
#include "custom_msgs/msg/velocity_req.hpp"
#include "custom_msgs/msg/velocity_cur.hpp"
#include "motor_driver/md_math.hpp"
#include "motor_driver/md_protocol.hpp"

#include <termios.h>
#include <unistd.h>
#include <iostream>

class MotorDriverNode : public rclcpp::Node
{
public:
    MotorDriverNode() : Node("motor_driver_node")
    {
        // VCU -> can_driver -> j1939_parser -> md_driver (torque request)
        vcu_to_md_sub_ = create_subscription<custom_msgs::msg::J1939Msg>(
            "/j1939_rx", 50,
            std::bind(&MotorDriverNode::callback, this, std::placeholders::_1));

        // md_driver -> j1939_parser -> can_driver -> vcu (cur RPM)
        md_to_vcu_pub_ = create_publisher<custom_msgs::msg::J1939Msg>("/j1939_tx", 50);
        
        // cur RPM publish timer
        md_to_vcu_send_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&MotorDriverNode::mdToVCUSendLoop, this));
        
        // md_driver -> issac sim (velocity request)
        velocity_req_pub_ = create_publisher<custom_msgs::msg::VelocityReq>("/velocity_req", 50);

        set_velocity_req_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&MotorDriverNode::setVelocityReqLoop, this));

        // issac sim -> md_driver (curent velocity)
        velocity_cur_sub_ = create_subscription<custom_msgs::msg::VelocityCur>(
            "/velocity_cur", 50,
            std::bind(&MotorDriverNode::velocityCurCallback, this, std::placeholders::_1));
    }

private:
    void callback(const custom_msgs::msg::J1939Msg::SharedPtr msg)
    {
        if(msg->pgn == MDProtocol::MD1toVCU_Protocol.pgn)
        {
            parseVCUtoMD(msg); 
        }
    }

    // VCU -> Motor torque request parsing
    void parseVCUtoMD(const custom_msgs::msg::J1939Msg::SharedPtr msg)
    {
        if (msg->dlc < 8) return;
        int16_t torque_req = msg->data[4] | (msg->data[5] << 8);

        switch(msg->source_address)
        {
            case MDProtocol::VCUtoMD1_Protocol.source_address:
                md1_cmd.torque_req = torque_req;
                break;
            case MDProtocol::VCUtoMD2_Protocol.source_address:
                md2_cmd.torque_req = torque_req;
                break;
            default:
                return;
        };

        RCLCPP_INFO(this->get_logger(), "MD1 Torque Cmd: %d, MD1 Torque Cmd: %d", md1_cmd.torque_req, md2_cmd.torque_req);
    }

    // cur velocity를 cur RPM으로 변환 후 전달
    void mdToVCUSendLoop()
    {
        publishMDtoVCU(MDProtocol::MD1toVCU_Protocol, md1_cur);
        publishMDtoVCU(MDProtocol::MD2toVCU_Protocol, md2_cur);
        RCLCPP_INFO(this->get_logger(), "MD1 RPM: %d MD2 RPM: %d", md1_cur.rpm_cur, md2_cur.rpm_cur);
    }

    void publishMDtoVCU(MDProtocol::ProtocolConfig md_protocol, MDProtocol::MDCur md_cur)
    {
        custom_msgs::msg::J1939Msg md_msg;

        md_msg.priority = md_protocol.priority;
        md_msg.pgn = md_protocol.pgn;
        md_msg.source_address = md_protocol.source_address;
        md_msg.dest_address = md_protocol.dest_address;
        md_msg.dlc = 8;
        
        // 현재 velocity -> rpm으로 변환 후 VCU에 전송
        md_cur.rpm_cur = MDMath::convertVelocityToRPM(md_cur.velocity_cur);

        md_msg.data[4] = md_cur.rpm_cur & 0xFF;
        md_msg.data[5] = (md_cur.rpm_cur >> 8) & 0xFF;

        md_to_vcu_pub_->publish(md_msg);
    }

    // torque req에 따라 acceleartion을 적용한 velocity req 전달
    void setVelocityReqLoop()
    {
        md1_cmd.velocity_req = MDMath::setVelocityReq(md1_cur.velocity_cur, md1_cmd.torque_req);
        md2_cmd.velocity_req = MDMath::setVelocityReq(md2_cur.velocity_cur, md2_cmd.torque_req);

        custom_msgs::msg::VelocityReq velocity_req;
        velocity_req.md1_velocity_req = md1_cmd.velocity_req;
        velocity_req.md2_velocity_req = md2_cmd.velocity_req;

        velocity_req_pub_->publish(velocity_req);
        RCLCPP_INFO(this->get_logger(), "MD1 Velocity Req: %d MD2 Velocity Req: %d", md1_cur.rpm_cur, md2_cur.rpm_cur);
    }

    // issac sim으로 부터 전달된 현재 velocity
    void velocityCurCallback(const custom_msgs::msg::VelocityCur msg)
    {
        md1_cur.velocity_cur = msg.md1_velocity_cur;
        md2_cur.velocity_cur = msg.md2_velocity_cur;
    }

private:
    rclcpp::Subscription<custom_msgs::msg::J1939Msg>::SharedPtr vcu_to_md_sub_;
    rclcpp::Publisher<custom_msgs::msg::J1939Msg>::SharedPtr md_to_vcu_pub_;
    rclcpp::Publisher<custom_msgs::msg::VelocityReq>::SharedPtr velocity_req_pub_;
    rclcpp::Subscription<custom_msgs::msg::VelocityCur>::SharedPtr velocity_cur_sub_;

    rclcpp::TimerBase::SharedPtr md_to_vcu_send_timer_;
    rclcpp::TimerBase::SharedPtr set_velocity_req_timer_;

    MDProtocol::MDCmd md1_cmd;
    MDProtocol::MDCur md1_cur;
    MDProtocol::MDCmd md2_cmd;
    MDProtocol::MDCur md2_cur;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorDriverNode>());
    rclcpp::shutdown();
    return 0;
}