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
#include "std_msgs/msg/u_int64.hpp"
#include "rmw/types.h"
#include "logging_custom.h"

using namespace std::chrono_literals;

constexpr bool IS_RELIABLE = true;
constexpr int DEFAULT_DEPTH = 5;
// PUBLISH_Hz는 Pong 노드에서 직접 사용하지 않지만, 일관성을 위해 유지합니다.
constexpr int PUBLISH_Hz = 10;
constexpr int DEFAULT_PORT = 8080;

constexpr const char *DESTINATION_IP = "192.168.1.205";

void setupTestFiles(std::vector<std::string> &msgs)
{
    int fileSizeBase = 131072;
    std::string fileNameSuffix = "_bytes.txt";

    while (true)
    {
        std::ifstream is{"./test_data/file_" + std::to_string(fileSizeBase) + fileNameSuffix};
        std::string buffer(std::istreambuf_iterator<char>(is), {});
        is >> buffer;

        msgs.push_back(buffer);

        fileSizeBase *= 2;
        is.close();

        if (fileSizeBase > 131072)
        {
            break;
        }
    }
}

class PingNode : public rclcpp::Node
{
public:
    PingNode() : Node("ping_node"), count_(0)
    {
        // --- QoS 설정: 상수를 사용하여 동적으로 구성 ---
        rclcpp::QoS qos_profile(1); // 기본값으로 초기화
        if (IS_RELIABLE) {
            // Reliable 설정: KEEP_ALL, 지정된 depth, RELIABLE, TRANSIENT_LOCAL
            qos_profile = rclcpp::QoS(rclcpp::KeepAll())
                              .reliable()
                              .transient_local();
        } else {
            // Best Effort 설정: KEEP_LAST(depth=1), BEST_EFFORT, VOLATILE
            qos_profile = rclcpp::QoS(rclcpp::KeepLast(DEFAULT_DEPTH))
                              .best_effort()
                              .durability_volatile();
        }
        // ---

        pong_subscriber_ = this->create_subscription<std_msgs::msg::UInt64>(
            "pong", qos_profile,
            std::bind(&PingNode::pong_callback, this, std::placeholders::_1));

        ping_publisher_ = this->create_publisher<std_msgs::msg::UInt64>("ping", qos_profile);
        
        // --- 타이머 주기: PUBLISH_Hz 상수를 사용하여 설정 ---
        auto timer_period = std::chrono::milliseconds(1000 / PUBLISH_Hz);
        timer_ = this->create_wall_timer(
            timer_period, std::bind(&PingNode::publish_ping, this));
        // ---

        rtt_results_.reserve(1000);
        RCLCPP_INFO(this->get_logger(), "Ping node started with %d Hz frequency.", PUBLISH_Hz);
    }

    ~PingNode()
    {
        RCLCPP_INFO(this->get_logger(), "Saving results to rt_rtt_results.txt");
        std::ofstream result_file("rt_rtt_results.txt");
        for (const auto& rtt : rtt_results_) {
            result_file << rtt << "\n";
        }
        result_file.close();
    }

private:
    void pong_callback(const std_msgs::msg::UInt64::SharedPtr msg)
    {
        struct timespec end_time_spec;
        clock_gettime(CLOCK_MONOTONIC, &end_time_spec);
        
        // timespec을 나노초(uint64_t)로 변환
        uint64_t end_ns = end_time_spec.tv_sec * BILLION + end_time_spec.tv_nsec;
        
        // RTT 계산 (현재 시간 - PING 보냈던 시간)
        uint64_t rtt = end_ns - msg->data;
        rtt_results_.push_back(rtt);
    }

    void publish_ping()
    {
        if (count_++ >= 1000) {
            RCLCPP_INFO(this->get_logger(), "Finished benchmarking.");
            rclcpp::shutdown();
            return;
        }

        // PING 메시지 전송 시간 측정
        struct timespec start_time_spec;
        clock_gettime(CLOCK_MONOTONIC, &start_time_spec);
        
        // timespec을 나노초(uint64_t)로 변환
        uint64_t start_ns = start_time_spec.tv_sec * BILLION + start_time_spec.tv_nsec;

        auto msg = std::make_unique<std_msgs::msg::UInt64>();
        msg->data = start_ns; // 메시지에 나노초 타임스탬프 기록
        ping_publisher_->publish(std::move(msg));
    }

    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr pong_subscriber_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr ping_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    int count_;
    std::vector<uint64_t> rtt_results_;
};

int main(int argc, char *argv[])
{
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall failed");
    }
    struct sched_param pri = {94};
    if (sched_setscheduler(0, SCHED_FIFO, &pri) == -1) {
        perror("sched_setscheduler failed");
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PingNode>());
    return 0;
}