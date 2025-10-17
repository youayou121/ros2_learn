#include<QApplication>
#include<QLabel>
#include<QString>
#include<rclcpp/rclcpp.hpp>
#include<status_interfaces/msg/system_status.hpp>
using SystemStatus = status_interfaces::msg::SystemStatus;
class StatusDisplayNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<SystemStatus>::SharedPtr subscriber_;
    std::shared_ptr<QLabel> label_;
public:
    StatusDisplayNode(const std::string &name_node):Node(name_node)
    {
        label_ = std::make_shared<QLabel>();
        label_->setText(QString::fromStdString("Waiting for status..."));
        label_->show();
        subscriber_ = this->create_subscription<SystemStatus>("/sys_status",10,
            std::bind(&StatusDisplayNode::topic_callback, this, std::placeholders::_1)
        );
    }
    void topic_callback(const SystemStatus::SharedPtr msg)
    {
        QString message = get_qstr_from_msg(msg);
        label_->setText(message);
    }

    QString get_qstr_from_msg(const SystemStatus::SharedPtr msg)
    {
        std::stringstream ss;
        ss << "------------------------system status display tool-----------------------\n";
        ss << "timestamp: \t\t" << msg->timestamp.sec << "s \t\n";
        ss << "host_name: \t\t" << msg->host_name << "\t\n";
        ss << "cpu_percent: \t\t" << msg->cpu_percent << "%\t\n";
        ss << "memory_percent: \t" << msg->memory_percent << "%\t\n";
        ss << "memory_total: \t\t" << msg->memory_total << "MB\t\n";
        ss << "memory_available: \t" << msg->memory_available << "MB\t\n";
        ss << "net_sent: \t\t" << msg->net_sent << "MB\t\n";
        ss << "net_recv: \t\t" << msg->net_recv << "MB\t\n";
        ss << "-----------------------------------------------------------------------\n";
        return QString::fromStdString(ss.str());
    }
};
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    rclcpp::init(argc, argv);
    std::shared_ptr<StatusDisplayNode> node = std::make_shared<StatusDisplayNode>("status_display_node");
    std::thread spin_thread([&]() -> void 
    {
        rclcpp::spin(node);
    });
    spin_thread.detach();
    app.exec();
    return 0;
}