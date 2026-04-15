#include "rclcpp/rclcpp.hpp"
#include "can_driver/can_driver.hpp"
#include "custom_msgs/msg/can_frame.hpp"

class Can0Node : public rclcpp::Node
{
public:
    Can0Node() : Node("can0_node")
    {
        driver_.init("can0");
        // ROS -> CAN
        sub_ = create_subscription<custom_msgs::msg::CanFrame>(
            "/can0_tx", 50,
            std::bind(&Can0Node::txCallback, this, std::placeholders::_1));

        // CAN -> ROS
        pub_ = create_publisher<custom_msgs::msg::CanFrame>("/can0_rx", 50);
        recv_thread_ = std::thread(&Can0Node::recvLoop, this);
    }

    ~Can0Node()
    {
        running_ = false;
        if (recv_thread_.joinable())
            recv_thread_.join();
    }

private:
    // tx
    void txCallback(const custom_msgs::msg::CanFrame::SharedPtr msg)
    {
        can_frame frame{};
        
        if (msg->is_extended) frame.can_id = (msg->id & CAN_EFF_MASK) | CAN_EFF_FLAG;
        else frame.can_id = msg->id;

        frame.can_dlc = msg -> dlc;
        for(int i=0; i<frame.can_dlc; i++)
        {
            frame.data[i] = msg -> data[i];
        }

        driver_.sendFrame(frame);
    }
    // rx
    void recvLoop()
    {
        struct can_frame frame;

        while (running_ && rclcpp::ok())
        {
            if (driver_.receiveFrame(frame))
            {
                custom_msgs::msg::CanFrame msg;
                msg.id = frame.can_id & CAN_EFF_MASK;
                msg.is_extended = frame.can_id & CAN_EFF_FLAG;
                msg.dlc = frame.can_dlc;
                for(int i=0; i<frame.can_dlc; i++)
                {
                    msg.data[i] = frame.data[i];
                }
                pub_->publish(msg);
            }
        }
    }

private:
    CanDriver driver_;

    rclcpp::Subscription<custom_msgs::msg::CanFrame>::SharedPtr sub_;
    rclcpp::Publisher<custom_msgs::msg::CanFrame>::SharedPtr pub_;

    std::thread recv_thread_;
    bool running_ = true;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Can0Node>());
    rclcpp::shutdown();
    return 0;
}