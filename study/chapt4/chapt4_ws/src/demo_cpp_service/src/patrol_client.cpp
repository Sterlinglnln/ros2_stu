#include <cstdlib>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "chapt4_interfaces/srv/patrol.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include <chrono>
using SetP = rcl_interfaces::srv::SetParameters;
using namespace std::chrono_literals;
using Patrol = chapt4_interfaces::srv::Patrol;

class PatrolClient : public rclcpp::Node {
public:
    PatrolClient() : Node("patrol_client") {
        patrol_client_ = this->create_client<Patrol>("/patrol");
        timer_ = this->create_wall_timer(10s, std::bind(&PatrolClient::timer_callback, this));
        srand(time(NULL)); // 初始化随机数种子,使用当前时间
    }

    void timer_callback() {
        // 1. 等待服务端上线
        while (!patrol_client_->wait_for_service(std::chrono::seconds(1))) {
            // 等待时监测 rclcpp 的状态
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被中断，退出。");
                return;
        }
            RCLCPP_INFO(this->get_logger(), "等待服务端上线...");
        }
        // 2. 构造请求的对象
        auto request = std::make_shared<Patrol::Request>();
        request->target_x = rand() % 15;  // 生成 0~14 的随机整数
        request->target_y = rand() % 15;
        RCLCPP_INFO(this->get_logger(), "请求巡逻点：(%.2f, %.2f)", request->target_x, request->target_y);
        // 3. 发布异步请求，然后等待返回，返回时调用回调函数
        patrol_client_->async_send_request(
            request,
            [&] (rclcpp::Client<Patrol>::SharedFuture result_future) -> void {
                auto response = result_future.get();
                if (response->result == Patrol::Response::SUCCESS) {
                    RCLCPP_INFO(this->get_logger(), "巡逻点设置成功，正在前往...");
                } else if (response->result == Patrol::Response::FAIL) {
                    RCLCPP_WARN(this->get_logger(), "巡逻点设置失败，请检查位置是否在边界内！");
                }
            });
    }

    void update_server_param_k(double k) {
        // 1. 创建一个参数对象
        auto param = rcl_interfaces::msg::Parameter();
        param.name = "k";
        // 2. 创建参数值对象并赋值
        auto param_value = rcl_interfaces::msg::ParameterValue();
        param_value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
        param_value.double_value = k;
        param.value = param_value;
        // 3. 请求更新参数并处理
        auto response = call_set_parameters(param);
        if (response == nullptr) {
            RCLCPP_WARN(this->get_logger(), "请求更新参数 k 失败！");
            return;
        } else {
            for (auto result : response->results) {
                if (result.successful) {
                    RCLCPP_INFO(this->get_logger(), "参数 k 已修改为 %f", k);
                } else {
                    RCLCPP_WARN(this->get_logger(), "参数 k 修改失败！原因：%s", result.reason.c_str());
                }
            }
        }
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<Patrol>::SharedPtr patrol_client_;

    std::shared_ptr<SetP::Response> call_set_parameters(rcl_interfaces::msg::Parameter &parameter) {
        // 1. 创建客户端等待服务上线
        auto param_client = this->create_client<SetP>("/turtle_controller/set_parameters");
        while (!param_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "等待参数服务的过程中被中断，退出。");
                return nullptr;
            }
            RCLCPP_INFO(this->get_logger(), "等待参数服务端上线...");
        }
        // 2. 构造请求对象
        auto request = std::make_shared<SetP::Request>();
        request->parameters.push_back(parameter);
        // 3. 异步发送请求并等待结果
        auto future = param_client->async_send_request(request);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);
        auto response = future.get();
        return response;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolClient>();
    // node->update_server_param_k(1.5); // 更新服务端参数 k 的值
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
