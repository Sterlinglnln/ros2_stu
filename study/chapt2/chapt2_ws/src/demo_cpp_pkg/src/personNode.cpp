#include "rclcpp/rclcpp.hpp"
using namespace std;

class personNode : public rclcpp::Node {
private:
    string name_;
    int age_;

public:
    personNode(const string &node_name,
        const string &name,
        const int &age) : Node(node_name) {    
        this->name_ = name;
        this->age_ = age;
    }
    
    void cook(const string &food_name) {
        RCLCPP_INFO(this->get_logger(), "I am %s, and I am %d years old. "
        "I am cooking %s.", name_.c_str(), age_, food_name.c_str());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = make_shared<personNode>("cpp_node", "Larry", 23);
    node->cook("ROS2");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
