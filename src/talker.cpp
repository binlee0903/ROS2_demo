#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <time.h>
#include <unistd.h>
#include <sched.h>
#include <arpa/inet.h>
#include <sys/mman.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rmw/types.h"

constexpr bool IS_RELIABLE = false;
constexpr int DEFAULT_DEPTH = 5;
constexpr int PUBLISH_Hz = 1000;
constexpr int DEFAULT_PORT = 8080;

constexpr const char* DESTINATION_IP = "192.168.1.11";

void setupTestFiles(std::vector<std::string>& msgs)
{
    int fileSizeBase = 256;
    std::string fileNameSuffix = ".txt";
    std::string buffer;
    buffer.reserve(16385); // 4MB

    while (fileSizeBase < 16385)
    {
        std::ifstream is { std::to_string(fileSizeBase) + fileNameSuffix };

        is >> buffer;

        msgs.push_back(buffer);

        fileSizeBase *= 2;
        is.close();
    }
}

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

int main(int argc, char **argv)
{
    mlockall(MCL_FUTURE);

    std::vector<std::string> msgs;
    std::vector<unsigned long long> publishTimes;
    std::vector<unsigned long long> subscribeTimes;
    setupTestFiles(msgs);
    publishTimes.reserve(120);
    subscribeTimes.reserve(120);

    sched_param param = {94};
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
        perror("sched_setattr");
        exit(EXIT_FAILURE);
    }

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("talker");

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

    auto default_publisher = node->create_publisher<std_msgs::msg::String>("default", qos_option);
    auto rate = rclcpp::WallRate::make_shared(PUBLISH_Hz);

    int sock = socket(AF_INET, SOCK_STREAM, 0);

    sockaddr_in sockaddr;
    sockaddr.sin_family = AF_INET;
    sockaddr.sin_port = htons(DEFAULT_PORT);
    sockaddr.sin_addr.s_addr = inet_addr(DESTINATION_IP);

    if (connect(sock, reinterpret_cast<struct sockaddr*>(&sockaddr), sizeof(sockaddr)) < 0)
    {
        printf("error on connection");
        return 1;
    }

    auto msg = std_msgs::msg::String();
    msg.data = "";
    int count = 1;
    int size = 0;
    int len = 0;
    char buffer[512];
    timespec time;

    for (auto x : msgs)
    {
        while (rclcpp::ok() == true)
        {
            size = x.size();
            printf("Test %d: msg size %d", count, size);

            msg.data = x;

            clock_gettime(CLOCK_REALTIME, &time);
            publishTimes.push_back(time.tv_sec + time.tv_nsec / 1000000000L);

            default_publisher->publish(msg);

            len = read(sock, buffer, sizeof(buffer));
            if (len <= 0)
            {
                return 1;
            }

            clock_gettime(CLOCK_REALTIME, &time);
            subscribeTimes.push_back(time.tv_sec + time.tv_nsec / 1000000000L);

            rclcpp::spin_some(node);
            rate->sleep();
            usleep(1000000);
        }
    }

    rclcpp::shutdown();

    std::ofstream os{"./output.txt"};

    int fileSizeBase = 256;

    for (int i = 0; i < subscribeTimes.size(); i++)
    {
        os << fileSizeBase << "bytes RTT: " << subscribeTimes[i] - publishTimes[i] << std::endl;
        fileSizeBase *= 2;
    }

    os.close();

    return 0;
}