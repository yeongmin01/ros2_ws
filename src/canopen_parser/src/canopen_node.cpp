#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/can_frame.hpp"
#include "custom_msgs/msg/canopen_msg.hpp"
#include "canopen_parser/canopen_parser.hpp"

class CANopenNode : public rclcpp::Node
{
public:
    CANopenNode() : Node("canopen_node")
    {
        // candriver -> canopen parser
        sub_can_ = create_subscription<custom_msgs::msg::CanFrame>(
            "/can1_rx", 50,
            std::bind(&CANopenNode::canCallback, this, std::placeholders::_1));
        // canopen parser -> other device
        pub_canopen_ = create_publisher<custom_msgs::msg::CanopenMsg>("/canopen_rx", 50);

        // other device -> canopen parser
        sub_canopen_ = create_subscription<custom_msgs::msg::CanopenMsg>(
            "/canopen_tx", 50,
            std::bind(&CANopenNode::canopenCallback, this, std::placeholders::_1));

        // canopen parser -> candriver
        pub_can_ = create_publisher<custom_msgs::msg::CanFrame>("/can1_tx", 50);
    }

private:
    void canCallback(const custom_msgs::msg::CanFrame::SharedPtr frame)
    {
        if (!CANopenParser::isCANopen(frame->id))
            return;

        auto msg = CANopenParser::parse(*frame);
        pub_canopen_->publish(msg);
    }

    void canopenCallback(const custom_msgs::msg::CanopenMsg::SharedPtr msg)
    {
        auto frame = CANopenParser::build(*msg);
        pub_can_->publish(frame);
    }

private:
    rclcpp::Subscription<custom_msgs::msg::CanFrame>::SharedPtr sub_can_;
    rclcpp::Publisher<custom_msgs::msg::CanopenMsg>::SharedPtr pub_canopen_;

    rclcpp::Subscription<custom_msgs::msg::CanopenMsg>::SharedPtr sub_canopen_;
    rclcpp::Publisher<custom_msgs::msg::CanFrame>::SharedPtr pub_can_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CANopenNode>());
    rclcpp::shutdown();
    return 0;
}