#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <tf2/utils.h>
using namespace std::chrono_literals;
using TFL = tf2_ros::TransformListener;
class TFListener : public rclcpp::Node
{
public:
    TFListener(const std::string &node_name) : Node(node_name)
    {
        timer_ = this->create_wall_timer(1000ms, std::bind(&TFListener::get_tf, this));
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<TFL>(*tf_buffer_);
    }
    void get_tf()
    {
        std::string target_frame = "map";
        std::string source_frame = "laser_link";
        try
        {
            const geometry_msgs::msg::TransformStamped tfs = tf_buffer_->lookupTransform(
                target_frame, source_frame, this->get_clock()->now(), rclcpp::Duration::from_seconds(1.0)
            );
            RCLCPP_INFO(this->get_logger(), "Got transform from %s in %s: translation(%.2f, %.2f, %.2f), rotation(%.2f, %.2f, %.2f, %.2f)",
                source_frame.c_str(), target_frame.c_str(), 
                tfs.transform.translation.x,
                tfs.transform.translation.y,
                tfs.transform.translation.z,
                tfs.transform.rotation.x,
                tfs.transform.rotation.y,
                tfs.transform.rotation.z,
                tfs.transform.rotation.w);
            double roll, pitch, yaw;
            tf2::getEulerYPR(tfs.transform.rotation, yaw, pitch, roll);
            RCLCPP_INFO(this->get_logger(), "Euler angles: roll: %.2f, pitch: %.2f, yaw: %.2f", roll, pitch, yaw);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not get transform: %s", e.what());
        }
    }
private:
    std::shared_ptr<TFL> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<TFListener> node = std::make_shared<TFListener>("tf_listener");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}