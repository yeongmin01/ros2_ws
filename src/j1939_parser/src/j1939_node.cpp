#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/msg/can_frame.hpp"
#include "custom_msgs/msg/j1939_msg.hpp"
#include "j1939_parser/j1939_parser.hpp"

class J1939Node : public rclcpp::Node
{
public:
    J1939Node() : Node("j1939_node")
    {
        // RX: CAN → J1939
        // can_driver -> j1939_parser
        sub_can_ = create_subscription<custom_msgs::msg::CanFrame>(
            "/can0_rx", 50,
            std::bind(&J1939Node::canCallback, this, std::placeholders::_1));
        // j1939_parser -> other j1939 device pkg
        pub_j1939_ = create_publisher<custom_msgs::msg::J1939Msg>("/j1939_rx", 50);

        // TX: J1939 → CAN
        // other j1939 device pkg -> j1939_parser
        sub_j1939_ = create_subscription<custom_msgs::msg::J1939Msg>(
            "/j1939_tx", 50,
            std::bind(&J1939Node::j1939Callback, this, std::placeholders::_1));
        // j1939_parser -> can_driver
        pub_can_ = create_publisher<custom_msgs::msg::CanFrame>("/can0_tx", 50);
    }

private:
    // 🔵 CAN → J1939
    void canCallback(const custom_msgs::msg::CanFrame::SharedPtr frame)
    {
        // 1. raw → j1939
        if (!J1939Parser::isJ1939(frame->id))
            return;

        auto j = J1939Parser::parse(*frame);
        pub_j1939_->publish(j);
    }

    // 🔴 J1939 → CAN
    void j1939Callback(const custom_msgs::msg::J1939Msg::SharedPtr msg)
    {
        // 1. j1939 → raw
        auto frame = J1939Parser::build(*msg);

        // 2. publish
        pub_can_->publish(frame);
    }

private:
    rclcpp::Subscription<custom_msgs::msg::CanFrame>::SharedPtr sub_can_;
    rclcpp::Publisher<custom_msgs::msg::J1939Msg>::SharedPtr pub_j1939_;

    rclcpp::Subscription<custom_msgs::msg::J1939Msg>::SharedPtr sub_j1939_;
    rclcpp::Publisher<custom_msgs::msg::CanFrame>::SharedPtr pub_can_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<J1939Node>());
    rclcpp::shutdown();
    return 0;
}