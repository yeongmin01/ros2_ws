#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/canopen_msg.hpp"
#include "steering_driver/steering_math.hpp"
#include "steering_driver/steering_protocol.hpp"
#include "custom_msgs/msg/steer_req.hpp"
#include "custom_msgs/msg/steer_cur.hpp"

#include <termios.h>
#include <unistd.h>
#include <iostream>

class SteeringDriverNode : public rclcpp::Node
{
public:
    SteeringDriverNode() : Node("motor_driver_node")
    {
        // VCU -> can_driver -> canopen_parser -> steer_driver (ampere request)
        vcu_to_steer_sub_ = create_subscription<custom_msgs::msg::CanopenMsg>(
            "/canopen_rx", 50,
            std::bind(&SteeringDriverNode::callback, this, std::placeholders::_1));

        // steer_driver -> canopen_parser -> can_driver -> vcu (cur AbsENC Position)
        steer_to_vcu_pub_ = create_publisher<custom_msgs::msg::CanopenMsg>("/canopen_tx", 50);
        
        // cur AbsENC Position publish timer
        steer_to_vcu_send_timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&SteeringDriverNode::steerToVCUSendLoop, this));
        
        // steer_driver -> Emulator manager (steering angle request)
        steer_req_pub_ = create_publisher<custom_msgs::msg::SteerReq>("/steer_req", 50);

        set_steer_req_timer_ = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&SteeringDriverNode::setSteerReqLoop, this));

        // Emulator manager -> steer_driver (curent steering angle)
        steer_cur_sub_ = create_subscription<custom_msgs::msg::SteerCur>(
            "/steer_cur", 50,
            std::bind(&SteeringDriverNode::steerCurCallback, this, std::placeholders::_1));
    }

private:
    // VCU -> steer_driver ampere request send
    void callback(const custom_msgs::msg::CanopenMsg::SharedPtr msg)
    {
        if(msg->function_code == SteeringProtocol::VCUtoFIO_Protocol.function_code &&
            msg->node_id == SteeringProtocol::VCUtoFIO_Protocol.node_id)
        {
            parseVCUtoSteer(msg); 
        }
    }

    // VCU -> steer current request parsing
    void parseVCUtoSteer(const custom_msgs::msg::CanopenMsg::SharedPtr msg)
    {
        if (msg->dlc < 8) return;

        steering_cmd.ampere_req[0] = msg->data[0] | (msg->data[1] << 8);
        steering_cmd.ampere_req[1] = msg->data[2] | (msg->data[3] << 8);
        steering_cmd.ampere_req[2] = msg->data[4] | (msg->data[5] << 8);
        steering_cmd.ampere_req[3] = msg->data[6] | (msg->data[7] << 8);

        //RCLCPP_INFO(this->get_logger(), "Ampere req: [%d, %d, %d, %d] ", steering_cmd.ampere_req[0] , steering_cmd.ampere_req[1],
        //                                                                    steering_cmd.ampere_req[2], steering_cmd.ampere_req[3]);
    }

    // cur steering angle -> AbsENC position 변환 후 전달
    void steerToVCUSendLoop()
    {
        // current steering angle -> AbsENC position 변환
        steering_cur.postion_cur = SteeringMath::convertWheelSteeringToPosition(steering_cur.wheel_steering_cur);

        for(int i = 0; i < 8; i++)
        {
            publishSteertoVCU(SteeringProtocol::AbsENC_RtoVCU_Protocol[i], steering_cur.postion_cur[i]);
        }

        //RCLCPP_INFO(this->get_logger(), "Current ENC Postion [%d, %d, %d, %d, %d, %d, %d, %d]", steering_cur.postion_cur[0], steering_cur.postion_cur[1],
        //                                                                                        steering_cur.postion_cur[2], steering_cur.postion_cur[3],
        //                                                                                        steering_cur.postion_cur[4], steering_cur.postion_cur[5],
        //                                                                                       steering_cur.postion_cur[6], steering_cur.postion_cur[7]);
    }

    void publishSteertoVCU(SteeringProtocol::ProtocolConfig steer_protocol, uint32_t position_cur)
    {
        custom_msgs::msg::CanopenMsg steer_msg;

        steer_msg.function_code = steer_protocol.function_code;
        steer_msg.node_id = steer_protocol.node_id;
        steer_msg.dlc = 8;

        steer_msg.data[0] = position_cur & 0xFF;
        steer_msg.data[1] = (position_cur >> 8) & 0xFF;
        steer_msg.data[2] = (position_cur >> 16) & 0xFF;
        steer_msg.data[3] = (position_cur >> 24) & 0xFF;
        
        steer_to_vcu_pub_->publish(steer_msg);
    }

    // ampere req에 따라 angular velocity를 적용한 steer angle req 전달
    void setSteerReqLoop()
    {
        steering_cmd.steering_req = SteeringMath::setSteeringReq(steering_cur.wheel_steering_cur, steering_cmd.ampere_req);

        custom_msgs::msg::SteerReq steer_req_msg;
        for(size_t i = 0; i < steering_cmd.steering_req.size(); i++)
        {
            steer_req_msg.steer_req[i] = steering_cmd.steering_req[i];
        }

        steer_req_pub_ -> publish(steer_req_msg);
        //RCLCPP_INFO(this->get_logger(), "Steer Req: [%d, %d, %d, %d]", steer_req, md2_cur.velocity_cur);
    }

    // issac sim으로 부터 전달된 현재 velocity
    void steerCurCallback(const custom_msgs::msg::SteerCur msg)
    {
        for(size_t i = 0; i < msg.steer_cur.size(); i ++)
        {
           steering_cur.wheel_steering_cur[i] = msg.steer_cur[i];
        }
    }

private:
    rclcpp::Subscription<custom_msgs::msg::CanopenMsg>::SharedPtr vcu_to_steer_sub_;
    rclcpp::Publisher<custom_msgs::msg::CanopenMsg>::SharedPtr steer_to_vcu_pub_;
    rclcpp::Publisher<custom_msgs::msg::SteerReq>::SharedPtr steer_req_pub_;
    rclcpp::Subscription<custom_msgs::msg::SteerCur>::SharedPtr steer_cur_sub_;

    rclcpp::TimerBase::SharedPtr steer_to_vcu_send_timer_;
    rclcpp::TimerBase::SharedPtr set_steer_req_timer_;

    SteeringProtocol::SteeringCmd steering_cmd = {{SteeringMath::zero_position_ampere, SteeringMath::zero_position_ampere, SteeringMath::zero_position_ampere, SteeringMath::zero_position_ampere}, {0.0f, 0.0f, 0.0f, 0.0f}};
    SteeringProtocol::SteeringCur steering_cur{};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SteeringDriverNode>());
    rclcpp::shutdown();
    return 0;
}