#include "optical_marker_tracking/marker_triangulator.hpp"
#include "optical_tracking_msgs/optical_tracking_utils.hpp"

#include <chrono>
#include <string>

using namespace std::chrono_literals;

MarkerTriangulator::MarkerTriangulator() : Node("marker_triangulator"),
    pmtx0_(initialize_pmtx(0)),
    pmtx2_(initialize_pmtx(2)),
    rmtx0_([this](){
        this->declare_parameter("rotation_matrix_0", {});
        std::vector<double> vec = this->get_parameter("rotation_matrix_0").as_double_array();
        return Eigen::Matrix3d(vec.data());                        
    }()),
    tvec0_([this](){        
        this->declare_parameter("translation_vector_0", {});
        std::vector<double> vec = this->get_parameter("translation_vector_0").as_double_array();
        return Eigen::Vector3d(vec.data());
    }())
{
    const auto QOS_BEKL5V = rclcpp::QoS(rclcpp::KeepLast(5))
        .best_effort().durability_volatile();

    marker_subscriber_ = this->create_subscription<PointMultiArray>(
        "/marker/data_raw", QOS_BEKL5V,
        [this](const PointMultiArray::SharedPtr msg){ marker_callback(msg); });

    marker_publisher_ = this->create_publisher<PointMultiArray>(
        "/marker/data", QOS_BEKL5V);
}                                                                  

MarkerTriangulator::~MarkerTriangulator()
{
}

Eigen::Matrix<double, 3, 4> MarkerTriangulator::initialize_pmtx(const int & number)
{
    this->declare_parameter("projection_matrix_" + std::to_string(number), {});
    auto vec = this->get_parameter("projection_matrix_" + std::to_string(number)).as_double_array();
    Eigen::Matrix<double, 4, 3> matrix;
    if (vec.size() == 12){
        Eigen::Map<const Eigen::Matrix<double, 4, 3>> map(vec.data());
        matrix = map;
    }
    return matrix.transpose();
}

void MarkerTriangulator::marker_callback(const PointMultiArray::SharedPtr msg)
{
    if (msg->data.size() <= 0)
        return;

    std::vector<Eigen::Vector3d> pts;
    pts.reserve(static_cast<int>(msg->data.size() / 2));

    Eigen::Vector3d Pw;
    std::vector<double> pt1, pt2;
    pt1.reserve(2);
    pt2.reserve(2);

    for (uint8_t i = 0; i < msg->data.size(); i+=2){
        pt1.push_back(msg->data[i].x);
        pt1.push_back(msg->data[i].y);

        pt2.push_back(msg->data[i + 1].x);
        pt2.push_back(msg->data[i + 1].y);

        triangulate_points(pt1, pt2, Pw);
        pts.push_back(Pw);

        pt1.clear();
        pt2.clear();
    }

    PointMultiArray point_msg;
    point_msg.stamp = this->get_clock()->now();
    optical_tracking_msgs::convert_vector_to_msg_3d(pts, point_msg.data);
    marker_publisher_->publish(point_msg);
}

void MarkerTriangulator::triangulate_points(
    const std::vector<double> & pt1,
    const std::vector<double> & pt2,
    Eigen::Vector3d & Pw
) {
    Eigen::Matrix4d A;
    A << pt1[1] * pmtx0_(2, Eigen::placeholders::all) - pmtx0_(1, Eigen::placeholders::all),
         pmtx0_(0, Eigen::placeholders::all) - pt1[0] * pmtx0_(2, Eigen::placeholders::all),
         pt2[1] * pmtx2_(2, Eigen::placeholders::all) - pmtx2_(1, Eigen::placeholders::all),
         pmtx2_(0, Eigen::placeholders::all) - pt2[0] * pmtx2_(2, Eigen::placeholders::all);

    const Eigen::Matrix4d B = A.transpose() * A;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(B, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::MatrixXd V = svd.matrixV();

    const Eigen::Vector3d Pc = V(Eigen::seq(0, 2), 3) / V(3, 3);
    Pw = rmtx0_ * (Pc - tvec0_);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto marker_triangulator = std::make_shared<MarkerTriangulator>();
    rclcpp::spin(marker_triangulator);
    rclcpp::shutdown();

    return 0;
}
