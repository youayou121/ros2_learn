#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
using namespace std::chrono_literals;
using TFB = tf2_ros::TransformBroadcaster;
class TFBroadcaster : public rclcpp::Node
{
public:
    TFBroadcaster(const std::string &node_name) : Node(node_name)
    {
        broadcaster_odom_in_map = std::make_shared<TFB>(this);
        timer_ = this->create_wall_timer(100ms, std::bind(&TFBroadcaster::publish_odom_in_map, this));
    }
    void publish_tf(std::shared_ptr<TFB> broadcaster, const std::string &parent_frame, const std::string &child_frame,
                    const double &x, const double &y, const double &z,
                    const double &roll, const double &pitch, const double &yaw)
    {
        geometry_msgs::msg::Transform tf;
        tf.translation.x = x;
        tf.translation.y = y;
        tf.translation.z = z;
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        tf.rotation = tf2::toMsg(q);
        geometry_msgs::msg::TransformStamped tfs;
        tfs.header.stamp = this->now();
        tfs.header.frame_id = parent_frame;
        tfs.child_frame_id = child_frame;
        tfs.transform = tf;
        broadcaster->sendTransform(tfs);
        RCLCPP_INFO(this->get_logger(), "Published tf %s in %s", child_frame.c_str(), parent_frame.c_str());
    }
    void publish_odom_in_map()
    {
        publish_tf(broadcaster_odom_in_map, "map", "odom",
                          1.0, 1.0, 1.0,
                          0.0, 0.0, 0.0);
    }
private:
    std::shared_ptr<TFB> broadcaster_odom_in_map;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<TFBroadcaster> node = std::make_shared<TFBroadcaster>("tf_broadcaster");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}