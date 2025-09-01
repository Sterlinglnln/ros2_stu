#include <cstdlib>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "chapt4_interfaces/srv/patrol.hpp"
#include <chrono>

using namespace std::chrono_literals;
using Patrol = chapt4_interfaces::srv::Patrol;

class PatrolClient : public rclcpp::Node {
public:
    PatrolClient() : Node("patrol_client") {
        patrol_client_ = this->create_client<Patrol>("parol");
        timer_ = this->create_wall_timer(10s, std::bind(&PatrolClient::timer_callback, this));
        srand(time(NULL)); // 初始化随机数种子,使用当前时间
    }
    void timer_callback() {
        // 1. 等待服务端上线
        while (!patrol_client_->wait_for_service(std::chrono::seconds(1))) {
            // 等待时监测 rclcpp 的状态
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
        }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        // 2. 构造请求的对象
        auto request = std::make_shared<Patrol::Request>();
        request->target_x = rand() % 15;  // 生成 0~14 的随机整数
        request->target_y = rand() % 15;
        RCLCPP_INFO(this->get_logger(), "Requesting patrol to (%.2f, %.2f)", request->target_x, request->target_y);
        // 3. 发布异步请求，然后等待返回，返回时调用回调函数
        patrol_client_->async_send_request(
            request,
            [&] (rclcpp::Client<Patrol>::SharedFuture result_future) -> void {
                auto response = result_future.get();
                if (response->result == Patrol::Response::SUCCESS) {
                    RCLCPP_INFO(this->get_logger(), "Patrol command accepted.");
                } else if (response->result == Patrol::Response::FAIL) {
                    RCLCPP_WARN(this->get_logger(), "Patrol command rejected.");
                }
            });
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<Patrol>::SharedPtr patrol_client_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}