#include "rclcpp/rclcpp.hpp"
#include "can_driver/can_driver.hpp"
//#include "j1939_parser/j1939_parser.hpp"

class CanNode : public rclcpp::Node
{
public:
    CanNode() : Node("can_node")
    {
        driver_.init("can0");
        // ROS -> CAN
        sub_ = create_subscription<can_frame>(
            "/can_tx", 50,
            std::bind(&CanNode::txCallback, this, std::placeholders::_1));

        // CAN -> ROS
        pub_ = create_publisher<can_frame>("/can_rx", 50);
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
    void txCallback(const can_frame frame)
    {
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
                pub_->publish(frame);
                
                RCLCPP_INFO(this->get_logger(),
                            "RX CAN ID: 0x%X DLC: %d",
                            frame.can_id, frame.can_dlc);
            }
        }
    }

private:
    CanDriver driver_;

    rclcpp::Subscription<can_frame>::SharedPtr sub_;
    rclcpp::Publisher<can_frame>::SharedPtr pub_;

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