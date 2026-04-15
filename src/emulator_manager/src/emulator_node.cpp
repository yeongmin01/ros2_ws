#include "rclcpp/rclcpp.hpp"

#include "custom_msgs/msg/velocity_req.hpp"
#include "custom_msgs/msg/velocity_cur.hpp"
#include "custom_msgs/msg/steer_req.hpp"
#include "custom_msgs/msg/steer_cur.hpp"

#include "emulator_manager/msg/steering_fb1.hpp"
#include "emulator_manager/msg/steering_fb2.hpp"
#include "emulator_manager/msg/steering_cmd.hpp"
#include "emulator_manager/msg/velocity_fb.hpp"
#include "emulator_manager/msg/velocity_cmd.hpp"

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
        emulator_to_issac_velocity_cmd_pub_ = create_publisher<emulator_manager::msg::VelocityCmd>("/velocity_cmd", 50);

        // emulator -> issac sim steering request
        emulator_to_issac_steering_cmd_pub_ = create_publisher<emulator_manager::msg::SteeringCmd>("/steering_cmd", 50);

        // issac sim -> emulator velocity feedback
        issac_to_emulator_velocity_fb_sub_ = create_subscription<emulator_manager::msg::VelocityFb>(
                                        "/velocity_fb", 50,
                                        std::bind(&EmulatorNode::velocityFBCallback, this, std::placeholders::_1));

        // issac sim -> emulator steering feedback1
        issac_to_emulator_Steering_fb1_sub_ = create_subscription<emulator_manager::msg::SteeringFb1>(
                                        "/steering_fb1", 50,
                                        std::bind(&EmulatorNode::steeringFB1Callback, this, std::placeholders::_1));

        // issac sim -> emulator steering feedback2
        issac_to_emulator_Steering_fb2_sub_ = create_subscription<emulator_manager::msg::SteeringFb2>(
                                        "/steering_fb2", 50,
                                        std::bind(&EmulatorNode::steeringFB2Callback, this, std::placeholders::_1));
    }

private:
    // md_driver -> emulator velocity request callback & publish VelocityCmd Msg to issac sim
    void velocityReqCallback(const custom_msgs::msg::VelocityReq::SharedPtr msg)
    {
        emulator_manager::msg::VelocityCmd velocity_cmd_msg;
        velocity_cmd_msg.velocity_cmd.data.resize(2);
        velocity_cmd_msg.velocity_cmd.data[0] = static_cast<int16_t>(msg -> md1_velocity_req * 100);
        velocity_cmd_msg.velocity_cmd.data[1] = static_cast<int16_t>(msg -> md2_velocity_req * 100);
    
        emulator_to_issac_velocity_cmd_pub_ -> publish(velocity_cmd_msg);
    }

    // steer_driver -> emulator steer request callback & publish SteeringCmd Msg to issac sim
    void steerReqCallback(const custom_msgs::msg::SteerReq::SharedPtr msg)
    {
        emulator_manager::msg::SteeringCmd steering_cmd_msg;
        steering_cmd_msg.steering_cmd.data.resize(4);
        for(size_t i = 0; i< msg->steer_req.size(); i++)
        {
            steering_cmd_msg.steering_cmd.data[i] = static_cast<int16_t>(msg -> steer_req[i] * 10);
        }

        emulator_to_issac_steering_cmd_pub_ -> publish(steering_cmd_msg);
    }

    void velocityFBCallback(const emulator_manager::msg::VelocityFb::SharedPtr msg)
    {
        custom_msgs::msg::VelocityCur velocity_cur_msg;
        velocity_cur_msg.md1_velocity_cur = static_cast<float>(msg -> velocity_fb.data[0]) / 100;
        velocity_cur_msg.md2_velocity_cur = static_cast<float>(msg -> velocity_fb.data[1]) / 100;

        velocity_cur_pub_ -> publish(velocity_cur_msg);
    }

    void steeringFB1Callback(const emulator_manager::msg::SteeringFb1::SharedPtr msg)
    {
        steer_cur_msg.steer_cur[0] = static_cast<float>(msg->steering_fb1.data[0])/10;
        steer_cur_msg.steer_cur[1] = static_cast<float>(msg->steering_fb1.data[1])/10;
        steer_cur_msg.steer_cur[2] = static_cast<float>(msg->steering_fb1.data[2])/10;
        steer_cur_msg.steer_cur[3] = static_cast<float>(msg->steering_fb1.data[3])/10;
        fb1_received_ = true;
        steeringFBPublish();
    }

    void steeringFB2Callback(const emulator_manager::msg::SteeringFb2::SharedPtr msg)
    {
        steer_cur_msg.steer_cur[4] = static_cast<float>(msg->steering_fb2.data[0])/10;
        steer_cur_msg.steer_cur[5] = static_cast<float>(msg->steering_fb2.data[1])/10;
        steer_cur_msg.steer_cur[6] = static_cast<float>(msg->steering_fb2.data[2])/10;
        steer_cur_msg.steer_cur[7] = static_cast<float>(msg->steering_fb2.data[3])/10;
        fb2_received_ = true;
        steeringFBPublish();
    }

    void steeringFBPublish()
    {
        if(fb1_received_ && fb2_received_)
        {
            steer_cur_pub_ -> publish(steer_cur_msg);
        }
    }

private:
    rclcpp::Publisher<custom_msgs::msg::VelocityCur>::SharedPtr velocity_cur_pub_;
    rclcpp::Subscription<custom_msgs::msg::VelocityReq>::SharedPtr velocity_req_sub_;
    rclcpp::Publisher<custom_msgs::msg::SteerCur>::SharedPtr steer_cur_pub_;
    rclcpp::Subscription<custom_msgs::msg::SteerReq>::SharedPtr steer_req_sub_;

    rclcpp::Publisher<emulator_manager::msg::SteeringCmd>::SharedPtr emulator_to_issac_steering_cmd_pub_;
    rclcpp::Publisher<emulator_manager::msg::VelocityCmd>::SharedPtr emulator_to_issac_velocity_cmd_pub_;
    rclcpp::Subscription<emulator_manager::msg::SteeringFb1>::SharedPtr issac_to_emulator_Steering_fb1_sub_;
    rclcpp::Subscription<emulator_manager::msg::SteeringFb2>::SharedPtr issac_to_emulator_Steering_fb2_sub_;
    rclcpp::Subscription<emulator_manager::msg::VelocityFb>::SharedPtr issac_to_emulator_velocity_fb_sub_;

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