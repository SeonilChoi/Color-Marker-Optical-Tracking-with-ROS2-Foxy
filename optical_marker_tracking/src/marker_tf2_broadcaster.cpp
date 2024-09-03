#include "optical_marker_tracking/marker_tf2_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <cstdint>
#include <string>

MarkerTF2Broadcaster::MarkerTF2Broadcaster() : Node("marker_tf2_broadcaster")
{
    const auto QOS_BEKL5V = rclcpp::QoS(rclcpp::KeepLast(5))
        .best_effort().durability_volatile();
    
    marker_subscriber_ = this->create_subscription<PoseStamped>(
        "object/data_raw", QOS_BEKL5V,
        [this](const PoseStamped::SharedPtr msg){ marker_callback(msg); });

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

MarkerTF2Broadcaster::~MarkerTF2Broadcaster()
{
}

void MarkerTF2Broadcaster::marker_callback(const PoseStamped::SharedPtr msg)
{
    
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "object";
    
    t.transform.translation.x = msg->pose.position.x;
    t.transform.translation.y = msg->pose.position.y;
    t.transform.translation.z = msg->pose.position.z;
    
    t.transform.rotation.x = msg->pose.orientation.x;
    t.transform.rotation.y = msg->pose.orientation.y;
    t.transform.rotation.z = msg->pose.orientation.z;
    t.transform.rotation.w = msg->pose.orientation.w;
    
    tf_broadcaster_->sendTransform(t);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto marker_tf2_broadcaster = std::make_shared<MarkerTF2Broadcaster>();
    rclcpp::spin(marker_tf2_broadcaster);
    rclcpp::shutdown();

    return 0;
}
