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
        
        // md_driver -> Emulator manager (velocity request)
        velocity_req_pub_ = create_publisher<custom_msgs::msg::VelocityReq>("/velocity_req", 50);

        set_velocity_req_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MotorDriverNode::setVelocityReqLoop, this));

        // Emulator manager -> md_driver (curent velocity)
        velocity_cur_sub_ = create_subscription<custom_msgs::msg::VelocityCur>(
            "/velocity_cur", 50,
            std::bind(&MotorDriverNode::velocityCurCallback, this, std::placeholders::_1));
    }

private:
    void callback(const custom_msgs::msg::J1939Msg::SharedPtr msg)
    {
        if(msg->pgn == MDProtocol::VCUtoMD1_Protocol1.pgn || msg->pgn == MDProtocol::VCUtoMD2_Protocol1.pgn) 
        {
            parseVCUtoMD1(msg); 
        }
        else if(msg->pgn == MDProtocol::VCUtoMD1_Protocol2.pgn || msg->pgn == MDProtocol::VCUtoMD2_Protocol2.pgn)
        {
            parseVCUtoMD2(msg); 
        }
        //RCLCPP_INFO(this->get_logger(), "MD1 [%d, %d, %d]", md1_cmd.torque_req, md1_cmd.gear_d, md1_cmd.gear_r);
        //RCLCPP_INFO(this->get_logger(), "MD2 [%d, %d, %d]", md2_cmd.torque_req, md2_cmd.gear_d, md2_cmd.gear_r);
    }

    // VCU -> Motor gear set request parsing
    void parseVCUtoMD1(const custom_msgs::msg::J1939Msg::SharedPtr msg)
    {
        if (msg->dlc < 8) return;

        bool gear_n = (msg->data[0] & (1 << 5)) != 0;
        bool gear_d = (msg->data[0] & (1 << 6)) != 0;
        bool gear_r = (msg->data[0] & (1 << 7)) != 0;

        switch(msg->source_address)
        {       
            case MDProtocol::VCUtoMD1_Protocol1.source_address:
                md1_cmd.gear_n = gear_n;    
                md1_cmd.gear_d = gear_d;
                md1_cmd.gear_r = gear_r;
                break;
            case MDProtocol::VCUtoMD2_Protocol1.source_address:
                md2_cmd.gear_n = gear_n; 
                md2_cmd.gear_d = gear_d;
                md2_cmd.gear_r = gear_r;
                break;
            default:
                return;
        };

        
    }

    // VCU -> Motor torque request parsing
    void parseVCUtoMD2(const custom_msgs::msg::J1939Msg::SharedPtr msg)
    {
        if (msg->dlc < 8) return;

        int16_t torque_req = static_cast<int16_t>(msg->data[4] | (msg->data[5] << 8));
        switch(msg->source_address)
        {
            case MDProtocol::VCUtoMD1_Protocol2.source_address:
                md1_cmd.torque_req = torque_req;
                RCLCPP_INFO(this->get_logger(), "MD1 [%d]", torque_req);
                break;
            case MDProtocol::VCUtoMD2_Protocol2.source_address:
                md2_cmd.torque_req = torque_req;
                RCLCPP_INFO(this->get_logger(), "MD2 [%d]", torque_req);
                break;
            default:
                return;
        };
    }

    // cur velocity를 cur RPM으로 변환 후 전달
    void mdToVCUSendLoop()
    {
        publishMDtoVCU(MDProtocol::MD1toVCU_Protocol4, MDProtocol::MD1toVCU_Protocol11, md1_cur);
        publishMDtoVCU(MDProtocol::MD2toVCU_Protocol4, MDProtocol::MD2toVCU_Protocol11, md2_cur);
        //RCLCPP_INFO(this->get_logger(), "MD1 RPM: %d MD2 RPM: %d", md1_cur.rpm_cur, md2_cur.rpm_cur);
    }

    void publishMDtoVCU(MDProtocol::ProtocolConfig md_protocol4, MDProtocol::ProtocolConfig md_protocol11, MDProtocol::MDCur md_cur)
    {
        custom_msgs::msg::J1939Msg md_msg4;

        md_msg4.priority = md_protocol4.priority;
        md_msg4.pgn = md_protocol4.pgn;
        md_msg4.source_address = md_protocol4.source_address;
        md_msg4.dest_address = md_protocol4.dest_address;
        md_msg4.dlc = 8;

        md_msg4.data[0] = 1; // bit 1 true

        md_to_vcu_pub_->publish(md_msg4);

        
        custom_msgs::msg::J1939Msg md_msg11;

        md_msg11.priority = md_protocol11.priority;
        md_msg11.pgn = md_protocol11.pgn;
        md_msg11.source_address = md_protocol11.source_address;
        md_msg11.dest_address = md_protocol11.dest_address;
        md_msg11.dlc = 8;
        
        // 현재 velocity -> rpm으로 변환 후 VCU에 전송
        md_cur.rpm_cur = MDMath::convertVelocityToRPM(md_cur.velocity_cur);
        
        md_msg11.data[4] = md_cur.rpm_cur & 0xFF;
        md_msg11.data[5] = (md_cur.rpm_cur >> 8) & 0xFF;

        md_to_vcu_pub_->publish(md_msg11);
    }

    // torque req에 따라 acceleartion을 적용한 velocity req 전달
    void setVelocityReqLoop()
    {
        md1_cmd.velocity_req = MDMath::setVelocityReq(md1_cmd, md1_cur.velocity_cur);
        md2_cmd.velocity_req = MDMath::setVelocityReq(md2_cmd, md2_cur.velocity_cur);

        custom_msgs::msg::VelocityReq velocity_req;
        velocity_req.md1_velocity_req = md1_cmd.velocity_req;
        velocity_req.md2_velocity_req = md2_cmd.velocity_req;

        velocity_req_pub_->publish(velocity_req);
        
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

    MDProtocol::MDCmd md1_cmd{};
    MDProtocol::MDCur md1_cur{};
    MDProtocol::MDCmd md2_cmd{};
    MDProtocol::MDCur md2_cur{};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorDriverNode>());
    rclcpp::shutdown();
    return 0;
}