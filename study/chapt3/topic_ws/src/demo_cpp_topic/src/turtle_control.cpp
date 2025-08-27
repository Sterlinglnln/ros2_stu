#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include <cmath>

class turtleController : public rclcpp::Node {
public:
    turtleController() : Node("turtle_controller") {
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10); // 创建发布者

        pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose", 10,
            std::bind(&turtleController::on_pose_received, this, std::placeholders::_1)
        ); // 创建订阅者

        // 初始化参数：目标位置、控制参数、停止状态
        target_x_ = 5.0;    // 目标位置 x 坐标
        target_y_ = 5.0;    // 目标位置 y 坐标
        k_ = 1.0;           // 比例系数
        max_speed_ = 2.0;   // 最大线速度
        distance_threshold_ = 0.05; // 停止阈值
        is_stopped_ = false; // 初始未停止
    }

private:
    // 位置回调函数：收到小海龟实时位置后，计算速度并发布
    void on_pose_received(const turtlesim::msg::Pose::SharedPtr pose) {
        // 如果已经停止，直接返回,不重复发布零速度
        if (is_stopped_) {
            return;
        }

        // 1.记录当前位置和朝向
        double current_x = pose->x;
        double current_y = pose->y; // 当前位置
        double current_theta = pose->theta; // 朝向角，弧度制
        RCLCPP_INFO_THROTTLE(  // 限制日志频率（1秒1次），避免刷屏
            this->get_logger(), *this->get_clock(), 1000,
            "当前位置: (%.2f, %.2f) | 距离目标: %.3f",
            current_x, current_y,
            std::sqrt(std::pow(target_x_ - current_x, 2) + std::pow(target_y_ - current_y, 2))
        );


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

    // ROS2 组件：订阅者（小海龟位置）、发布者（速度指令）
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;

    // 控制参数
    double target_x_;          // 目标位置X坐标
    double target_y_;          // 目标位置Y坐标
    double k_;                 // 线速度比例系数
    double max_speed_;         // 最大线速度
    double distance_threshold_;// 到达目标的距离阈值（越小精度越高）
    bool is_stopped_;          // 停止状态标记（避免重复发布零速度）
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv); // 初始化ROS2
    auto controller_node = std::make_shared<turtleController>(); // 创建节点
    rclcpp::spin(controller_node); // 运行节点，进入循环等待回调函数
    rclcpp::shutdown(); // 关闭ROS2
    return 0;
}
