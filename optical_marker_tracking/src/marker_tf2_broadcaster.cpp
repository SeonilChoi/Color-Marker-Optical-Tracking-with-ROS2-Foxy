#include "optical_marker_tracking/marker_tf2_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <cstdint>
#include <string>

MarkerTF2Broadcaster::MarkerTF2Broadcaster() : Node("marker_tf2_broadcaster")
{
    const auto QOS_BEKL5V = rclcpp::QoS(rclcpp::KeepLast(5))
        .best_effort().durability_volatile();

    marker_subscriber_ = this->create_subscription<PointMultiArray>(
        "/marker/data", QOS_BEKL5V,
        [this](const PointMultiArray::SharedPtr msg){ marker_callback(msg); });

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

MarkerTF2Broadcaster::~MarkerTF2Broadcaster()
{
}

void MarkerTF2Broadcaster::marker_callback(const PointMultiArray::SharedPtr msg)
{
    for (size_t i = 0; i < msg->data.size(); i++){
        geometry_msgs::msg::TransformStamped t;
        
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "table";
        t.child_frame_id = "object" + std::to_string(i);
      
        t.transform.translation.x = msg->data[i].x;
        t.transform.translation.y = msg->data[i].y;
        t.transform.translation.z = msg->data[i].z;

        tf_broadcaster_->sendTransform(t);
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto marker_tf2_broadcaster = std::make_shared<MarkerTF2Broadcaster>();
    rclcpp::spin(marker_tf2_broadcaster);
    rclcpp::shutdown();

    return 0;
}

