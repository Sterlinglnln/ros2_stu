#include <iostream>
#include "motion_control_system/spin_motion_controller.hpp"
namespace motion_control_system {
    void SpinMotionController::start() {
        // 实现启动旋转运动的逻辑
        std::cout << "SpinMotionController started spinning." << std::endl;
    }
    
    void SpinMotionController::stop() {
        // 实现停止旋转运动的逻辑
        std::cout << "SpinMotionController stopped spinning." << std::endl;
    }
}   // namespace motion_control_system

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(motion_control_system::SpinMotionController, motion_control_system::MotionController)