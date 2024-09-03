#include "optical_marker_tracking/marker_tracker.hpp"
#include "optical_tracking_msgs/optical_tracking_utils.hpp"

#include <cmath>

const double m_pi = 3.141592654;

MarkerTracker::MarkerTracker() : Node("marker_tracker"),
    Rw_(initialize_parameter()),
    tw_([this](){
        this->declare_parameter("world_translation_vector", {});
        std::vector<double> vec = this->get_parameter("world_translation_vector").as_double_array();
        return Eigen::Vector3d(vec.data());
    }()),
    Ro_([this](){
        this->declare_parameter("object_rotation_matrix", {});
        std::vector<double> vec = this->get_parameter("object_rotation_matrix").as_double_array();
        
        std::vector<Eigen::Matrix3d> m;
        for (size_t i = 0; i < 4; i++){
            std::vector<double> data;
            std::copy(vec.begin() + (i * 9), vec.begin() + (i * 9 + 9), std::back_inserter(data));
            m.push_back(Eigen::Matrix3d(data.data()).transpose());
        }
        return m;
    }()),
    to_([this](){
        this->declare_parameter("object_translation_vector", {});
        std::vector<double> vec = this->get_parameter("object_translation_vector").as_double_array();
        
        std::vector<Eigen::Vector3d> m;
        for (size_t i = 0; i < 4; i++){
            std::vector<double> data;
            std::copy(vec.begin() + (i * 3), vec.begin() + (i * 3 + 3), std::back_inserter(data));
            m.push_back(Eigen::Vector3d(data.data()));
        }
        return m;
    }()),
    marker_geometry_([this](){
        this->declare_parameter("marker_geometry", {});
        std::vector<double> vec = this->get_parameter("marker_geometry").as_double_array();
        
        std::vector<Eigen::Vector3d> m;
        for (size_t i = 0; i < 4; i++){
            std::vector<double> data;
            std::copy(vec.begin() + (i * 3), vec.begin() + (i * 3 + 3), std::back_inserter(data));
            m.push_back(Eigen::Vector3d(data.data()));
        }
        return m;
    }()),
    marker_geometry_angle_([this](){
        this->declare_parameter("marker_geometry_angle", {});
        std::vector<double> vec = this->get_parameter("marker_geometry_angle").as_double_array();
        
        std::vector<Eigen::Vector3d> m;
        for (size_t i = 0; i < 4; i++){
            std::vector<double> data;
            std::copy(vec.begin() + (i * 3), vec.begin() + (i * 3 + 3), std::back_inserter(data));
            m.push_back(Eigen::Vector3d(data.data()));
        }
        return m;
    }())

{
    const auto QOS_BEKL5V = rclcpp::QoS(rclcpp::KeepLast(5))
        .best_effort().durability_volatile();

    marker_subscriber_ = this->create_subscription<PointMultiArray>(
        "/marker/data", QOS_BEKL5V,
        [this](const PointMultiArray::SharedPtr msg){ marker_callback(msg); });
    
    pose_publisher_ = this->create_publisher<PoseStamped>(
        "/object/data_raw", QOS_BEKL5V);
}

MarkerTracker::~MarkerTracker()
{
}

Eigen::Matrix3d MarkerTracker::initialize_parameter()
{
    this->declare_parameter("world_rotation_matrix", {});
    auto vec = this->get_parameter("world_rotation_matrix").as_double_array();
    Eigen::Matrix3d matrix;
    if (vec.size() == 9){
        Eigen::Map<const Eigen::Matrix3d> map(vec.data());
        matrix = map;
    }
    return matrix.transpose();
}

void MarkerTracker::marker_callback(const PointMultiArray::SharedPtr msg)
{
    if (msg->data.size() < 3)
        return;
    
    std::vector<Eigen::Vector3d> init_points, points;
    optical_tracking_msgs::convert_msg_to_vector_3d(msg->data, init_points);
    
    uint8_t idx;
    find_marker_geometry(init_points, idx, points);

    if (points.empty())
        return;

    reconstruct_pose(idx, points);
}

void MarkerTracker::find_marker_geometry(
    const std::vector<Eigen::Vector3d> & in_pts,
    uint8_t & idx,
    std::vector<Eigen::Vector3d> & out_pts
) {   
    const uint8_t in_pts_size = in_pts.size();
    std::vector<uint8_t> init_index(in_pts_size);
    std::iota(init_index.begin(), init_index.end(), 0);
    
    double min_error = 1e+5;
    std::vector<uint8_t> min_error_index;
    min_error_index.reserve(3);
    permutation(init_index, 0, in_pts_size, 3, in_pts, idx, min_error, min_error_index);

    out_pts.reserve(3);
    for (const auto & idx : min_error_index){
        out_pts.push_back(in_pts[idx]);
    }
}  

void MarkerTracker::permutation(
    std::vector<uint8_t> data,
    const uint8_t & depth,
    const uint8_t & n,
    const uint8_t & r,
    const std::vector<Eigen::Vector3d> & in_pts,
    uint8_t & idx,
    double & min_error,
    std::vector<uint8_t> & min_error_index
) {
    if (depth == r){
        const Eigen::Vector3d v1 = in_pts[data[1]] - in_pts[data[0]];
        const Eigen::Vector3d v2 = in_pts[data[2]] - in_pts[data[0]];
        const Eigen::Vector3d v3 = in_pts[data[1]] - in_pts[data[2]];
        const double v1_norm = v1.norm();
        const double v2_norm = v2.norm();
        const double v3_norm = v3.norm();
        const double angle1 = std::acos(v1.dot(v2) / (v1_norm * v2_norm));
        const double angle2 = std::acos(-v1.dot(-v3) / (v1_norm * v3_norm));
        const double angle3 = std::acos(-v2.dot(v3) / (v2_norm * v3_norm));

        const Eigen::Vector3d current_geometry {v1_norm, v2_norm, v3_norm};
        const Eigen::Vector3d current_geometry_angle {angle1, angle2, angle3};
        
        uint8_t idx_per_one_step;
        double min_error_per_one_step = 1e+5;
        for (uint8_t i = 0; i < 4; i++){
            double error = (current_geometry - marker_geometry_angle_[i]).norm()
                + (current_geometry_angle - marker_geometry_angle_[i]).norm();
            if (error < min_error_per_one_step){
                idx_per_one_step = i;
                min_error_per_one_step = error;
            }
        }
        
        if (min_error_per_one_step < min_error){
            idx = idx_per_one_step;
            min_error = min_error_per_one_step;
            min_error_index.assign(data.begin(), data.begin() + r);
        }
   
        return;
    }

    for (uint8_t i = depth; i < n; i++){
        std::swap(data[depth], data[i]);
        permutation(data, depth+1, n, r, in_pts, idx, min_error, min_error_index);
        std::swap(data[i], data[depth]);
    }
}

void MarkerTracker::reconstruct_pose(
    const uint8_t & idx,
    const std::vector<Eigen::Vector3d> & in_pts
) {
    const Eigen::Vector3d v1 = in_pts[1] - in_pts[0];
    const Eigen::Vector3d v2 = in_pts[2] - in_pts[0];

    const Eigen::Vector3d z = v1.cross(v2);
    Eigen::Vector3d x, y;
    if (idx == 2){
        y = v2;
        x = y.cross(z);
    } else {
        x = v1;
        y = z.cross(x);
    }

    Eigen::Matrix3d R;
    R(Eigen::placeholders::all, 0) = x / x.norm();
    R(Eigen::placeholders::all, 1) = y / y.norm();
    R(Eigen::placeholders::all, 2) = z / z.norm();

    Eigen::Vector3d t = in_pts[0] + R * to_[idx];
    t = tw_ + Rw_ * t;

    Eigen::Vector4d q;
    convert_rotation_matrix_to_quaternion(Rw_ * R * Ro_[idx], q);

    publish_point_msg(t, q);
}

void MarkerTracker::convert_rotation_matrix_to_quaternion(
    const Eigen::Matrix3d & R,
    Eigen::Vector4d & q
) {
    double t = R.trace();

    if (t > 0){
        t = std::sqrt(t + 1);
        q(3) = 0.5 * t;
        t = 0.5 / t;
        q(0) = (R(2, 1) - R(1, 2)) * t;
        q(1) = (R(0, 2) - R(2, 0)) * t;
        q(2) = (R(1, 0) - R(0, 1)) * t;
    }
    else{
        uint8_t i = 0;
        if (R(1, 1) > R(0, 0))
            i = 1;
        if (R(2, 2) > R(i, i))
            i = 2;
        uint8_t j = (i + 1) % 3;
        uint8_t k = (j + 1) % 3;

        t = std::sqrt(R(i, i) - R(j, j) - R(k, k) + 1);
        q(i) = 0.5 * t;
        t = 0.5 / t;
        q(3) = (R(k, j) - R(j, k)) * t;
        q(j) = (R(j, i) + R(i, j)) * t;
        q(k) = (R(k, i) + R(i, k)) * t;
    }
}

void MarkerTracker::publish_point_msg(
    const Eigen::Vector3d & position,
    const Eigen::Vector4d & orientation    
) {
    PoseStamped msg;
    msg.header.stamp = this->get_clock()->now();

    msg.pose.position.x = position(0);
    msg.pose.position.y = position(1);
    msg.pose.position.z = position(2);

    msg.pose.orientation.x = orientation(0);
    msg.pose.orientation.y = orientation(1);
    msg.pose.orientation.z = orientation(2);
    msg.pose.orientation.w = orientation(3);
    
    pose_publisher_->publish(msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto marker_tracker = std::make_shared<MarkerTracker>();
    rclcpp::spin(marker_tracker);
    rclcpp::shutdown();

    return 0;
}
