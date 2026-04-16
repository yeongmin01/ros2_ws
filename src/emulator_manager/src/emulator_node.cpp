#include "rclcpp/rclcpp.hpp"

#include "custom_msgs/msg/velocity_req.hpp"
#include "custom_msgs/msg/velocity_cur.hpp"
#include "custom_msgs/msg/steer_req.hpp"
#include "custom_msgs/msg/steer_cur.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"

#include <termios.h>
#include <unistd.h>
#include <iostream>

class EmulatorNode : public rclcpp::Node
{
public:
    EmulatorNode() : Node("motor_driver_node")
    {
        // md_driver -> emulator velocity request
        velocity_req_sub_ = create_subscription<custom_msgs::msg::VelocityReq>(
            "/velocity_req", 50,
            std::bind(&EmulatorNode::velocityReqCallback, this, std::placeholders::_1));

        // emulator -> md_driver velocity current
        velocity_cur_pub_ = create_publisher<custom_msgs::msg::VelocityCur>("/velocity_cur", 50);

        // steer_driver -> emulator steer angle request
        steer_req_sub_ = create_subscription<custom_msgs::msg::SteerReq>(
            "/steer_req", 50,
            std::bind(&EmulatorNode::steerReqCallback, this, std::placeholders::_1));

        // emulator -> steer driver steer angle current
        steer_cur_pub_ = create_publisher<custom_msgs::msg::SteerCur>("/steer_cur", 50);
        
        // emulator -> issac sim velocity request
        emulator_to_issac_velocity_cmd_pub_ = create_publisher<std_msgs::msg::Int16MultiArray>("/vehicle/velocity_cmd", 50);

        // emulator -> issac sim steering request
        emulator_to_issac_steering_cmd_pub_ = create_publisher<std_msgs::msg::Int16MultiArray>("/vehicle/steering_cmd", 50);

        // issac sim -> emulator velocity feedback
        issac_to_emulator_velocity_fb_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
                                        "/Isaac/velocity_fb", 50,
                                        std::bind(&EmulatorNode::velocityFBCallback, this, std::placeholders::_1));

        // issac sim -> emulator steering feedback1
        issac_to_emulator_Steering_fb1_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
                                        "/Isaac/steering_fb1", 50,
                                        std::bind(&EmulatorNode::steeringFB1Callback, this, std::placeholders::_1));

        // issac sim -> emulator steering feedback2
        issac_to_emulator_Steering_fb2_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
                                        "/Isaac/steering_fb2", 50,
                                        std::bind(&EmulatorNode::steeringFB2Callback, this, std::placeholders::_1));
    }

private:
    // md_driver -> emulator velocity request callback & publish VelocityCmd Msg to issac sim
    void velocityReqCallback(const custom_msgs::msg::VelocityReq::SharedPtr msg)
    {
        std_msgs::msg::Int16MultiArray velocity_cmd_msg;
        velocity_cmd_msg.data.resize(2);
        velocity_cmd_msg.data[0] = static_cast<int16_t>(msg -> md1_velocity_req * 100);
        velocity_cmd_msg.data[1] = static_cast<int16_t>(msg -> md2_velocity_req * 100);
    
        emulator_to_issac_velocity_cmd_pub_ -> publish(velocity_cmd_msg);
    }

    // steer_driver -> emulator steer request callback & publish SteeringCmd Msg to issac sim
    void steerReqCallback(const custom_msgs::msg::SteerReq::SharedPtr msg)
    {
        std_msgs::msg::Int16MultiArray steering_cmd_msg;
        steering_cmd_msg.data.resize(4);
        for(size_t i = 0; i< msg->steer_req.size(); i++)
        {
            steering_cmd_msg.data[i] = static_cast<int16_t>(msg -> steer_req[i] * 10);
        }

        emulator_to_issac_steering_cmd_pub_ -> publish(steering_cmd_msg);
    }

    void velocityFBCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
    {
        custom_msgs::msg::VelocityCur velocity_cur_msg;
        velocity_cur_msg.md1_velocity_cur = static_cast<float>(msg -> data[0]) / 100;
        velocity_cur_msg.md2_velocity_cur = static_cast<float>(msg -> data[1]) / 100;

        velocity_cur_pub_ -> publish(velocity_cur_msg);
    }

    void steeringFB1Callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
    {
        steer_cur_msg.steer_cur[0] = static_cast<float>(msg->data[0])/10;
        steer_cur_msg.steer_cur[1] = static_cast<float>(msg->data[1])/10;
        steer_cur_msg.steer_cur[2] = static_cast<float>(msg->data[2])/10;
        steer_cur_msg.steer_cur[3] = static_cast<float>(msg->data[3])/10;
        fb1_received_ = true;
        steeringFBPublish();
    }

    void steeringFB2Callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
    {
        steer_cur_msg.steer_cur[4] = static_cast<float>(msg->data[0])/10;
        steer_cur_msg.steer_cur[5] = static_cast<float>(msg->data[1])/10;
        steer_cur_msg.steer_cur[6] = static_cast<float>(msg->data[2])/10;
        steer_cur_msg.steer_cur[7] = static_cast<float>(msg->data[3])/10;
        fb2_received_ = true;
        steeringFBPublish();
    }

    void steeringFBPublish()
    {
        // steering fb1 ,2 가 모두 들어오면 steering driver로 pub
        if(fb1_received_ && fb2_received_)
        {
            steer_cur_pub_ -> publish(steer_cur_msg);
            fb1_received_ = false;
            fb2_received_ = false;
        }
    }

private:
    rclcpp::Publisher<custom_msgs::msg::VelocityCur>::SharedPtr velocity_cur_pub_;
    rclcpp::Subscription<custom_msgs::msg::VelocityReq>::SharedPtr velocity_req_sub_;
    rclcpp::Publisher<custom_msgs::msg::SteerCur>::SharedPtr steer_cur_pub_;
    rclcpp::Subscription<custom_msgs::msg::SteerReq>::SharedPtr steer_req_sub_;

    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr emulator_to_issac_steering_cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr emulator_to_issac_velocity_cmd_pub_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr issac_to_emulator_Steering_fb1_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr issac_to_emulator_Steering_fb2_sub_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr issac_to_emulator_velocity_fb_sub_;

    custom_msgs::msg::SteerCur steer_cur_msg{};
    bool fb1_received_ = false;
    bool fb2_received_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EmulatorNode>());
    rclcpp::shutdown();
    return 0;
}