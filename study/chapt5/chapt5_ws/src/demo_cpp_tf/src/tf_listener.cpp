#include <memory>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "chrono"

using namespace std::chrono_literals;

class TFListenerNode : public rclcpp::Node {
private:
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    TFListenerNode() : Node("tf_listener") {
        buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_, this);
        timer_ = this->create_wall_timer(5s, std::bind(&TFListenerNode::getTransform, this));
    }

    void getTransform() {
        try {
            const auto transform = buffer_->lookupTransform(
                "base_link", "target_point", this->get_clock()->now(), rclcpp::Duration::from_seconds(1.0f));
            const auto& translation = transform.transform.translation;
            const auto& rotation = transform.transform.rotation;
            double roll, pitch, yaw;
            tf2::getEulerYPR(rotation, yaw, pitch, roll);
            RCLCPP_INFO(this->get_logger(), "Translation: (%f, %f, %f)", translation.x, translation.y, translation.z);
            RCLCPP_INFO(this->get_logger(), "Rotation (RPY): (%f, %f, %f)", roll, pitch, yaw);
        } catch (tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform: %s", ex.what());
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFListenerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
