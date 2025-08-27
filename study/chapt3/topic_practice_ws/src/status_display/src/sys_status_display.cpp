#include <QApplication>
#include <QLabel>
#include <QString>
#include <sstream>
#include <thread>
#include <QMetaObject>  // 用于线程安全更新 UI
#include "rclcpp/rclcpp.hpp"
#include "status_interfaces/msg/system_status.hpp"

using SystemStatus = status_interfaces::msg::SystemStatus;

class SysStatusDisplay : public rclcpp::Node {
public:
    SysStatusDisplay(QLabel* label) : Node("sys_status_display"), label_(label) {
        if (!label_) {
            RCLCPP_ERROR(this->get_logger(), "Label 指针为空！");
            return;
        }

        // 订阅话题，使用 bind 保证 this 指针正确
        subscription_ = this->create_subscription<SystemStatus>(
            "system_status", 10, 
            std::bind(&SysStatusDisplay::topicCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "系统状态显示节点已初始化");
    }

private:
    // ROS 话题回调（处理线程安全）
    void topicCallback(const SystemStatus::SharedPtr msg) {
        if (!label_) return;

        // 转换消息为 QString（调用格式化函数）
        QString displayText = formatMsgToQString(msg);
        
        // 线程安全更新 UI：通过 Qt 元对象系统投递事件
        QMetaObject::invokeMethod(label_, "setText", 
            Qt::QueuedConnection, 
            Q_ARG(QString, displayText)
        );
    }

    // 格式化消息为可读字符串（修复单位、时间显示）
    QString formatMsgToQString(const SystemStatus::SharedPtr msg) {
        std::stringstream showStr;
        showStr 
            << "==============系统状态可视化工具==============\n"
            // 改用 Python 发布端的 readable_stamp（如果定义了）
            << "时间：\t" << msg->readable_stamp.c_str() << "\n"  
            << "主机名：\t" << msg->host_name.c_str() << "\n"
            << "CPU使用率：\t" << msg->cpu_percent << "%\n"
            << "内存使用率：\t" << msg->memory_percent << "%\n"
            << "内存总大小：\t" << msg->memory_total << " GB\n"
            << "剩余有效内存：\t" << msg->memory_available << " GB\n"
            << "网络发送量：\t" << msg->net_sent << " MB\n"
            << "网络接收量：\t" << msg->net_recv << " MB\n"
            << "============================================";

        return QString::fromStdString(showStr.str());
    }

    rclcpp::Subscription<SystemStatus>::SharedPtr subscription_;
    QLabel* label_;  // 关联 Qt 显示标签
};

int main(int argc, char *argv[]) {
    // 初始化 ROS 2
    rclcpp::init(argc, argv);
    
    // 初始化 Qt 应用
    QApplication app(argc, argv);
    QLabel label;
    label.setWindowTitle("系统状态监控");
    label.resize(400, 300);  // 设置窗口大小
    label.show();  // 先显示空窗口，等待数据

    // 创建节点（传入 Qt Label 指针）
    auto node = std::make_shared<SysStatusDisplay>(&label);

    // 在独立线程运行 ROS 循环
    std::thread spinThread([&node]() {
        rclcpp::spin(node);
    });
    spinThread.detach();

    // 运行 Qt 事件循环
    int appResult = app.exec();

    // 清理资源
    rclcpp::shutdown();
    // （可选）若需要严格线程管理，可 join，但 detach 场景下一般无需
    // spinThread.join();  

    return appResult;
}