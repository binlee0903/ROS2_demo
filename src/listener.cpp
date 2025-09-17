#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <time.h>
#include <unistd.h>
#include <sched.h>
#include <arpa/inet.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

constexpr bool IS_RELIABLE = true;
constexpr int DEFAULT_DEPTH = 5;
constexpr int PUBLISH_Hz = 1000;
constexpr int DEFAULT_PORT = 8080;

int _socketClient = 0;
int connectCount = 0;

static const rmw_qos_profile_t rmw_qos_profile_reliable = {
    RMW_QOS_POLICY_HISTORY_KEEP_ALL,
    5,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL};

static const rmw_qos_profile_t rmw_qos_profile_best_effort = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE};


void callback(const std_msgs::msg::String::SharedPtr msg){
    write(_socketClient, "x", 1);
    connectCount++;

    if (connectCount >= 10)
    {
        connectCount = 0;
        close(_socketClient);
        rclcpp::shutdown();
    }
}

int main(int argc, char** argv)
{
    sockaddr_in sockaddrin, sockaddrClient;
    int socketServer = socket(AF_INET, SOCK_STREAM, 0);
    unsigned int clientSize = sizeof(sockaddrClient);

    sockaddrin.sin_family = AF_INET;
    sockaddrin.sin_port = htons(DEFAULT_PORT);
    sockaddrin.sin_addr.s_addr = INADDR_ANY;

    int trueValue = 1;
    setsockopt(socketServer, SOL_SOCKET, SO_REUSEADDR, &trueValue, sizeof(int));

    bind(socketServer, reinterpret_cast<struct sockaddr*>(&sockaddrin), sizeof(sockaddrin));

    listen(socketServer, 100);

    for (int i = 0; i < 10; i++)
    {
        _socketClient = accept(socketServer, reinterpret_cast<struct sockaddr*>(&sockaddrClient), &clientSize);
        rclcpp::init(argc, argv);

        auto node = rclcpp::Node::make_shared("listener");
        rclcpp::QoS qos_option(DEFAULT_DEPTH);
        if (IS_RELIABLE == true)
        {
            qos_option
                .reliable()
                .keep_all()
                .transient_local();
        }
        else
        {
            qos_option
                .best_effort()
                .keep_all()
                .durability_volatile();
        }

        auto subscriber = node->create_subscription<std_msgs::msg::String>("listener",  qos_option, callback);

        rclcpp::spin(node);
    }

    close(socketServer);

    return 0;
}