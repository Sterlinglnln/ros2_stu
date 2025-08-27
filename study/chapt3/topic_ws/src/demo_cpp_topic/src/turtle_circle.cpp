#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
using namespace std::chrono_literals; // 使用时间字面量
using namespace std;

class turtleCircle : public rclcpp::Node {
private:
    rclcpp::TimerBase::SharedPtr timer_; // 定时器智能指针
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; // 发布者智能指针

    void timer_callback() {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 1.0; // 线速度
        message.angular.z = 0.5; // 角速度
        RCLCPP_INFO(this->get_logger(), "Publishing: linear.x: '%f', angular.z: '%f'",
            message.linear.x, message.angular.z);
        publisher_->publish(message); // 发布消息
    }

public:
    explicit turtleCircle(const string& node_name) : Node(node_name) {
        // 调用继承自 Node 的 create_publisher 方法创建发布者
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        // 调用继承自 Node 的 create_wall_timer 方法创建定时器
        timer_ = this->create_wall_timer(
            1000ms, bind(&turtleCircle::timer_callback, this)
        );
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv); // 初始化 ROS 2
    auto node = make_shared<turtleCircle>("turtle_circle"); // 创建节点
    rclcpp::spin(node); // 运行节点，进入循环等待回调函数
    rclcpp::shutdown(); // 关闭 ROS 2
    return 0;
}
