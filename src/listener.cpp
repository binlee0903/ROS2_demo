#include <memory>
#include <sched.h>
#include <sys/mman.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"

// --- 설정용 상수 ---
constexpr bool IS_RELIABLE = true;
constexpr int DEFAULT_DEPTH = 5;
// ---

// Pong 노드는 받은 메시지를 그대로 되돌려주는 역할만 합니다.
class PongNode : public rclcpp::Node
{
public:
    PongNode() : Node("pong_node")
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
            qos_profile = rclcpp::QoS(rclcpp::KeepLast(1))
                              .best_effort()
                              .durability_volatile();
        }
        // ---

        // '/ping' 토픽을 구독합니다.
        ping_subscriber_ = this->create_subscription<std_msgs::msg::UInt64>(
            "ping", qos_profile,
            std::bind(&PongNode::ping_callback, this, std::placeholders::_1));

        // '/pong' 토픽으로 발행합니다.
        pong_publisher_ = this->create_publisher<std_msgs::msg::UInt64>("pong", qos_profile);

        RCLCPP_INFO(this->get_logger(), "Pong node has started. Waiting for pings.");
    }

private:
    void ping_callback(const std_msgs::msg::UInt64::SharedPtr msg)
    {
        pong_publisher_->publish(*msg);
    }

    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr ping_subscriber_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr pong_publisher_;
};

int main(int argc, char *argv[])
{
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall failed");
    }
    struct sched_param pri = {90};
    if (sched_setscheduler(0, SCHED_FIFO, &pri) == -1) {
        perror("sched_setscheduler failed");
    }

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PongNode>());
    rclcpp::shutdown();
    return 0;
}