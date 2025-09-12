#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "chapt4_interfaces/srv/patrol.hpp"
#include <cmath>
#include "rcl_interfaces/msg/set_parameters_result.hpp"
using SetParametersResult = rcl_interfaces::msg::SetParametersResult;

using Patrol = chapt4_interfaces::srv::Patrol;

class turtleController : public rclcpp::Node {
public:
    turtleController() : Node("turtle_controller") {
        // 声明参数
        this->declare_parameter("k", 1.0);
        this->declare_parameter("max_speed", 2.0);
        this->get_parameter("k", k_);
        this->get_parameter("max_speed", max_speed_);

        // 创建发布者和订阅者
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10,
            std::bind(&turtleController::on_pose_received, this, std::placeholders::_1)
        );

        // 初始化参数：目标位置、控制参数、停止状态
        k_ = 1.0;           // 比例系数
        max_speed_ = 2.0;   // 最大线速度
        distance_threshold_ = 0.05; // 停止阈值
        is_stopped_ = false; // 初始未停止
        is_target_set_ = false; // 初始无目标

        // 创建服务
        patrol_server_ = this->create_service<Patrol>(
            "/patrol",
            [&] (const std::shared_ptr<Patrol::Request> request,
                std::shared_ptr<Patrol::Response> response) -> void {
            // 判断巡逻点是否在模拟器边界内
            if ((0 < request->target_x && request->target_x < 12.0f)
                && (0 < request->target_y && request->target_y < 12.0f)) {
                    target_x_ = request->target_x;
                    target_y_ = request->target_y;
                    is_target_set_ = true; // 设置目标成功
                    is_stopped_ = false; // 重置停止状态，允许继续运动
                    response->result = Patrol::Response::SUCCESS;
                    RCLCPP_INFO(this->get_logger(), "已接收目标巡逻位置: (%.2f, %.2f)，开始前往！", target_x_, target_y_);
                } else {
                    response->result = Patrol::Response::FAIL;
                    RCLCPP_WARN(this->get_logger(), "目标位置 (%.2f, %.2f) 超出边界，拒绝执行！", request->target_x, request->target_y);
                }
            });
        
        // 添加参数设置回调
        parameters_callback_handle_ = this->add_on_set_parameters_callback(
            [&] (const std::vector<rclcpp::Parameter> &params)
            -> SetParametersResult {
                // 遍历所有参数，更新k和max_speed
                for (auto param : params) {
                    RCLCPP_INFO(this->get_logger(), "更新参数 %s 值为：%f", param.get_name().c_str(), param.as_double());
                    if (param.get_name() == "k") {
                        k_ = param.as_double();
                    } else if (param.get_name() == "max_speed") {
                        max_speed_ = param.as_double();
                    }
                }
                auto result = SetParametersResult();
                result.successful = true;
                return result;
        });
    }

private:
    // 服务、订阅、发布对象和参数回调句柄
    rclcpp::Service<Patrol>::SharedPtr patrol_server_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;

    // 控制参数
    double target_x_;          // 目标位置X坐标
    double target_y_;          // 目标位置Y坐标
    double k_;                 // 线速度比例系数
    double max_speed_;         // 最大线速度
    double distance_threshold_;// 到达目标的距离阈值（越小精度越高）
    bool is_stopped_;          // 停止状态标记（避免重复发布零速度）
    bool is_target_set_;       // 是否已设置目标位置

    // 位置回调函数：收到小海龟实时位置后，计算速度并发布
    void on_pose_received(const turtlesim::msg::Pose::SharedPtr pose) {
        // 如果未设置目标，直接返回
        if (!is_target_set_) {
            return;
        }

        // 如果已经停止，直接返回,不重复发布零速度
        if (is_stopped_) {
            return;
        }

        // 1.记录当前位置和朝向
        double current_x = pose->x;
        double current_y = pose->y; // 当前位置
        double current_theta = pose->theta; // 朝向角，弧度制

        // 2.计算核心误差：距离误差和角度误差
        double distance_error = std::sqrt(
            std::pow(target_x_ - current_x, 2) + std::pow(target_y_ - current_y, 2)
        );
        double target_angle = std::atan2(target_y_ - current_y, target_x_ - current_x); // 目标角度
        double angle_error = target_angle - current_theta;
        // 归一化角度到[-π, π]
        angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));

        // 3.控制逻辑：根据误差决定速度
        auto velocity_msg = geometry_msgs::msg::Twist();  // 速度指令消息

        if (distance_error > distance_threshold_) {
            // 3.1 未到达目标：按误差计算速度
            if (std::fabs(angle_error) > 0.1) {  // 角度误差大：优先旋转对准目标（阈值0.1弧度≈5.7度）
                velocity_msg.angular.z = 2.0 * angle_error;  // 角速度与角度误差成正比（系数2可调整）
            } else {  // 角度误差小：前进趋近目标
                velocity_msg.linear.x = k_ * distance_error;  // 线速度与距离误差成正比
                // 限制最大线速度，避免冲过头
                if (velocity_msg.linear.x > max_speed_) {
                    velocity_msg.linear.x = max_speed_;
                }
            }
        } else {
            // 3.2 到达目标：发布零速度，标记为“已停止”
            velocity_msg.linear.x = 0.0;
            velocity_msg.angular.z = 0.0;
            is_stopped_ = true;  // 标记停止状态，后续回调不再处理
            RCLCPP_WARN(this->get_logger(), "已到达目标位置！停止运动！");
        }

        // 4.发布速度指令（无论运动还是停止）
        velocity_publisher_->publish(velocity_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv); // 初始化ROS2
    auto controller_node = std::make_shared<turtleController>(); // 创建节点
    rclcpp::spin(controller_node); // 运行节点，进入循环等待回调函数
    rclcpp::shutdown(); // 关闭ROS2
    return 0;
}
