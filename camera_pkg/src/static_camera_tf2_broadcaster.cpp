#include "camera_pkg/static_camera_tf2_broadcaster.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <string>
#include <cstdint>
#include <vector>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/LinearMath/Quaternion.h"

StaticCameraTF2Broadcaster::StaticCameraTF2Broadcaster()
    : Node("static_camera_tf2_broadcaster")
{
    this->declare_parameter("parent_link", "world");
    std::string parent_link = this->get_parameter("parent_link")
        .get_value<std::string>();
    
    tf_static_broadcaster_ = std::make_shared
        <tf2_ros::StaticTransformBroadcaster>(this);

    send_static_transform(parent_link, 0);
    send_static_transform(parent_link, 2);
}

StaticCameraTF2Broadcaster::~StaticCameraTF2Broadcaster()
{
}

void StaticCameraTF2Broadcaster::send_static_transform(const std::string & parent_link, const uint8_t & camera_number)
{
    this->declare_parameter("rotation_matrix_cam" + std::to_string(camera_number), {});
    std::vector<double> R = this->get_parameter("rotation_matrix_cam" + std::to_string(camera_number))
        .get_value<std::vector<double>>();
    
    this->declare_parameter("translation_vector_cam" + std::to_string(camera_number), {});
    std::vector<double> T = this->get_parameter("translation_vector_cam" + std::to_string(camera_number))
        .get_value<std::vector<double>>();

    tf2::Matrix3x3 Rmtx(R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8]);
    tf2::Vector3 Tvec(T[0], T[1], T[2]);

    Rmtx = Rmtx.transpose();
    Tvec = Rmtx * -Tvec;

    tf2::Quaternion q;
    Rmtx.getRotation(q);

    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = parent_link;
    t.child_frame_id = "camera" + std::to_string(camera_number);
    
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    t.transform.translation.x = Tvec.getX();
    t.transform.translation.y = Tvec.getY();
    t.transform.translation.z = Tvec.getZ();

    tf_static_broadcaster_->sendTransform(t);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto static_camera_tf2_broadcaster = std::make_shared
        <StaticCameraTF2Broadcaster>();
    rclcpp::spin(static_camera_tf2_broadcaster);
    rclcpp::shutdown();

    return 0;
}
