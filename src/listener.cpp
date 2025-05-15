#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class HelloworldSubscriber : public rclcpp::Node
{
public:
    HelloworldSubscriber()
        : Node("hello_world_subscriber")
    {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        hello_world_subscriber = this->create_subscription<std_msgs::msg::String>("helloworld", qos_profile,
        std::bind(&HelloworldSubscriber::subscribe_message, this, _1));
    }

    void subscribe_message(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
    }

    
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr hello_world_subscriber;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<HelloworldSubscriber>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}