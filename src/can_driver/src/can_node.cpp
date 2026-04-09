#include "rclcpp/rclcpp.hpp"
#include "can_driver/can_driver.hpp"
#include "custom_msgs/msg/can_frame.hpp"

class CanNode : public rclcpp::Node
{
public:
    CanNode() : Node("can_node")
    {
        driver_.init("can0");
        // ROS -> CAN
        sub_ = create_subscription<custom_msgs::msg::CanFrame>(
            "/can_tx", 50,
            std::bind(&CanNode::txCallback, this, std::placeholders::_1));

        // CAN -> ROS
        pub_ = create_publisher<custom_msgs::msg::CanFrame>("/can_rx", 50);
        recv_thread_ = std::thread(&CanNode::recvLoop, this);
    }

    ~CanNode()
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
        
        frame.can_id = msg -> id;
        frame.can_dlc = msg -> dlc;
        for(int i=0; i<frame.can_dlc; i++)
        {
            frame.data[i] = msg -> data[i];
        }

        driver_.sendFrame(frame);

         RCLCPP_INFO(this->get_logger(),
                    "TX CAN ID: 0x%X DLC: %d",
                    frame.can_id, frame.can_dlc);
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
                msg.id = frame.can_id;
                msg.dlc = frame.can_dlc;
                for(int i=0; i<frame.can_dlc; i++)
                {
                    msg.data[i] = frame.data[i];
                }
                pub_->publish(msg);
                
                RCLCPP_INFO(this->get_logger(),
                            "RX CAN ID: 0x%X DLC: %d",
                            frame.can_id, frame.can_dlc);
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
    rclcpp::spin(std::make_shared<CanNode>());
    rclcpp::shutdown();
    return 0;
}