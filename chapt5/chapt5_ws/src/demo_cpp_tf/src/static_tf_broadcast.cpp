#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
using STFB = tf2_ros::StaticTransformBroadcaster;
class StaticTFBroadcaster : public rclcpp::Node
{
public:
    StaticTFBroadcaster(const std::string &node_name) : Node(node_name)
    {
        static_broadcaster_laser_in_base = std::make_shared<STFB>(this);
        static_broadcaster_base_in_odom = std::make_shared<STFB>(this);
    }
    void publish_static_tf(std::shared_ptr<STFB> broadcaster, const std::string &parent_frame, const std::string &child_frame,
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
        RCLCPP_INFO(this->get_logger(), "Published static tf %s in %s", child_frame.c_str(), parent_frame.c_str());
    }
    void publish_laser_in_base()
    {
        publish_static_tf(static_broadcaster_laser_in_base, "base_link", "laser_link",
                          1.0, 0.0, 0.0,
                          0.0, 0.0, 0.0);
    }
    void publish_base_in_odom()
    {
        publish_static_tf(static_broadcaster_base_in_odom, "odom", "base_link",
                          0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0);
    }
private:
    std::shared_ptr<STFB> static_broadcaster_laser_in_base;
    std::shared_ptr<STFB> static_broadcaster_base_in_odom;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<StaticTFBroadcaster> node = std::make_shared<StaticTFBroadcaster>("static_tf_broadcaster");
    node->publish_laser_in_base();
    node->publish_base_in_odom();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}