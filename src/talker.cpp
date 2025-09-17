#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include <time.h>
#include <unistd.h>
#include <sched.h>
#include <arpa/inet.h>
#include <sched.h>
#include <sys/mman.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rmw/types.h"

constexpr bool IS_RELIABLE = true;
constexpr int DEFAULT_DEPTH = 5;
constexpr int PUBLISH_Hz = 10;
constexpr int DEFAULT_PORT = 8080;

constexpr const char *DESTINATION_IP = "192.168.1.205";

void setupTestFiles(std::vector<std::string> &msgs)
{
    int fileSizeBase = 256;
    std::string fileNameSuffix = "_bytes.txt";

    while (true)
    {
        std::ifstream is{"./test_data/file_" + std::to_string(fileSizeBase) + fileNameSuffix};
        std::string buffer(std::istreambuf_iterator<char>(is), {});
        is >> buffer;

        msgs.push_back(buffer);

        fileSizeBase *= 2;
        is.close();

        if (fileSizeBase > 524288)
        {
            break;
        }
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

    usleep(1000); // avoid race condition
    sched_param pri = {94};
    if (sched_setscheduler(0, SCHED_FIFO, &pri) == -1)
    {
        perror("sched_setattr");
        exit(EXIT_FAILURE);
    }

    for (int j = 0; j < 10; j++)
    {
        std::vector<std::string> msgs;
        std::vector<unsigned long long> publishTimes;
        std::vector<unsigned long long> subscribeTimes;
        setupTestFiles(msgs);
        publishTimes.reserve(120);
        subscribeTimes.reserve(120);

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

        auto default_publisher = node->create_publisher<std_msgs::msg::String>("listener", qos_option);
        auto rate = rclcpp::WallRate::make_shared(PUBLISH_Hz);

        int sock = socket(AF_INET, SOCK_STREAM, 0);

        sockaddr_in sockaddr = {0};
        sockaddr.sin_family = AF_INET;
        sockaddr.sin_port = htons(DEFAULT_PORT);
        inet_pton(AF_INET, DESTINATION_IP, &sockaddr.sin_addr);

        if (connect(sock, reinterpret_cast<struct sockaddr *>(&sockaddr), sizeof(sockaddr)) < 0)
        {
            printf("error on connection");
            perror("connect");
            return 1;
        }

        auto msg = std_msgs::msg::String();
        msg.data = "h";
        int count = 1;
        int size = 0;
        int len = 0;
        char buffer[512];
        timespec time;
        unsigned long long publishTime = 0;
        unsigned long long subscribeTime = 0;

        int i = 0;

        // Skip the internal initialization process.
        while (rclcpp::ok())
        {
            if (i++ > 20)
            {
                break;
            }
            default_publisher->publish(msg);

            len = read(sock, buffer, sizeof(buffer));
            if (len <= 0)
            {
                return 1;
            }

            rclcpp::spin_some(node);
            rate->sleep();
        }

        usleep(1000000);

        // real evaluation
        for (auto x : msgs)
        {
            size = x.size();
            printf("Test %d: msg size %d \n", count++, size);

            msg.data = x;

            if (clock_gettime(CLOCK_MONOTONIC_RAW, &time) < 0)
            {
                printf("clock error");
            }
            publishTime = time.tv_sec * 1000000000L + time.tv_nsec;

            default_publisher->publish(msg);

            len = read(sock, buffer, sizeof(buffer));
            if (len <= 0)
            {
                return 1;
            }

            if (clock_gettime(CLOCK_MONOTONIC_RAW, &time) < 0)
            {
                printf("clock error");
            }

            subscribeTime = time.tv_sec * 1000000000L + time.tv_nsec;

            publishTimes.push_back(publishTime);
            subscribeTimes.push_back(subscribeTime);
            rclcpp::spin_some(node);
            rate->sleep();
            usleep(1000000);
        }

        rclcpp::shutdown();

        std::ofstream os{"./output.txt"};

        int fileSizeBase = 256;

        for (int i = 0; i < subscribeTimes.size(); i++)
        {
            os << fileSizeBase << "bytes RTT: " << subscribeTimes[i] - publishTimes[i] << "ns" << std::endl;
            fileSizeBase *= 2;
        }

        os.close();
        close(sock);
        usleep(1000000);
    }

    return 0;
}