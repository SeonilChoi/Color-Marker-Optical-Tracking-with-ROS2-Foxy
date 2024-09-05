#include "optical_marker_tracking/marker_matcher.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <chrono>
#include <cmath>
#include <numeric>
#include <algorithm> 

std::vector<std::vector<double>> vec_to_mat(
    const std::vector<double> & data,
    const int & rows,
    const int & cols
) {
    std::vector<std::vector<double>> mat(rows, std::vector<double>(cols));
    for (int i = 0; i < rows; i++){
        for (int j = 0; j < cols; j++){
            mat[i][j] = data[i * cols + j];
        }
    }
    return mat;
}

void subtract(
    const std::vector<double> & a,
    std::vector<double> & b
) {
    for (size_t i = 0; i < a.size(); i++){
        b[i] = a[i] - b[i];
    }
}

void square(std::vector<double> & a)
{
    for (auto & val : a){
        val *= val;
    }
}

double sum(const std::vector<double> & a)
{
    return std::accumulate(a.begin(), a.end(), 0.0);
}

std::vector<double> divide(
    const std::vector<double> & a,
    const std::vector<double> & b
) {
    std::vector<double> c(a.size());
    std::transform(a.begin(), a.end(), b.begin(), c.begin(), std::divides<double>());
    return c;
}

void divide(
    std::vector<std::vector<double>> & a,
    const double & b
) {
    for (auto & row : a){
        for (auto & val : row){
            val /= b;
        }
    }
}

void add(
    const std::vector<std::vector<double>> & a,
    const std::vector<std::vector<double>> & b,
    std::vector<std::vector<double>> & c
) {
    const size_t m = a.size();
    const size_t n = a[0].size();
    
    if (!c.empty())
        c.clear();

    c.resize(m, std::vector<double>(n, 0.0));

    for (size_t i = 0; i < m; i++){
        for (size_t j = 0; j < n; j++){
            c[i][j] = a[i][j] + b[i][j];
        }
    }
}

void add(
    const std::vector<std::vector<double>> & a,
    std::vector<std::vector<double>> & b
) {
    for (size_t i = 0; i < a.size(); i++){
        for (size_t j = 0; j < a[0].size(); j++){
            b[i][j] = a[i][j] + b[i][j];
        }
    }
}

std::vector<std::vector<double>> dot(
    const std::vector<std::vector<double>> & a,
    const std::vector<std::vector<double>> & b
) {
    const size_t m = a.size();
    const size_t n = b[0].size();
    const size_t inner = b.size();

    std::vector<std::vector<double>> c(m, std::vector<double>(n, 0.0));

    for (size_t i = 0; i < m; i++){
        for (size_t j = 0; j < n; j++){
            for (size_t k = 0; k < inner; k++){
                c[i][j] += (a[i][k] * b[k][j]);
            }
        }
    }
    return c;
}

void tanh(std::vector<std::vector<double>> & a)
{
    for (auto & row : a){
        for (auto & val : row){
            double exp_x = std::exp(val);
            double exp_neg_x = 1 / exp_x;
            val = (exp_x - exp_neg_x) / (exp_x + exp_neg_x);
        }
    }    
}

void permutation(
    std::vector<uint8_t> data,
    const uint8_t & depth,
    const uint8_t & n,
    const uint8_t & r,
    const std::vector<std::vector<double>> & error_table,
    double & min_error,
    std::vector<uint8_t> & min_error_index
) {
    if (depth == r){
        double error = 0.0;
        for (uint8_t i = 0; i < r; i++)
            error += error_table[i][data[i]];

        if (error < min_error){
            min_error = error;
            min_error_index.assign(data.begin(), data.begin() + r);
        }

        return;
    }

    for (uint8_t i = depth; i < n; i++){
        std::swap(data[depth], data[i]);
        permutation(data, depth+1, n, r, error_table, min_error, min_error_index);
        std::swap(data[i], data[depth]);
    }
}

using namespace std::chrono_literals;

const double nano2sec = 1e-9;

MarkerMatcher::MarkerMatcher() : Node("marker_matcher"),
    cmtx1_([this](){
        this->declare_parameter("intrinsic_matrix_0", {});
        std::vector<double> data = this->get_parameter("intrinsic_matrix_0").as_double_array();
        return cv::Mat(3, 3, CV_64F, data.data()).clone();        
    }()),
    dist1_([this](){
        this->declare_parameter("distortion_coefficients_0", {});
        std::vector<double> data = this->get_parameter("distortion_coefficients_0").as_double_array();
        return cv::Mat(1, 5, CV_64F, data.data()).clone();     
    }()),
    cmtx2_([this](){
        this->declare_parameter("intrinsic_matrix_2", {});
        std::vector<double> data = this->get_parameter("intrinsic_matrix_2").as_double_array();
        return cv::Mat(3, 3, CV_64F, data.data()).clone();        
    }()),
    dist2_([this](){
        this->declare_parameter("distortion_coefficients_2", {});
        std::vector<double> data = this->get_parameter("distortion_coefficients_2").as_double_array();
        return cv::Mat(1, 5, CV_64F, data.data()).clone();     
    }()),
    fmtx_([this](){
        this->declare_parameter("fundamental_matrix", {});
        std::vector<double> data = this->get_parameter("fundamental_matrix").as_double_array();
        return cv::Mat(3, 3, CV_64F, data.data()).clone();        
    }()),
    w1_([this](){
        this->declare_parameter("fc1_weights", {});
        std::vector<double> data = this->get_parameter("fc1_weights").as_double_array();
        return vec_to_mat(data, 3, 8);
    }()),
    b1_([this](){
        this->declare_parameter("fc1_bias", {});
        std::vector<double> data = this->get_parameter("fc1_bias").as_double_array();
        return vec_to_mat(data, 1, 8); 
    }()),
    w2_([this](){
        this->declare_parameter("fc2_weights", {});
        std::vector<double> data = this->get_parameter("fc2_weights").as_double_array();
        return vec_to_mat(data, 8, 3);
    }()),
    b2_([this](){
        this->declare_parameter("fc2_bias", {});
        std::vector<double> data = this->get_parameter("fc2_bias").as_double_array();
        return vec_to_mat(data, 1, 3); 
    }()),
    maximum_value_([this](){
        this->declare_parameter("maximum_value", {});
        return this->get_parameter("maximum_value").as_double_array();        
    }())
{
    const auto QOS_BEKL5V = rclcpp::QoS(rclcpp::KeepLast(5))
        .best_effort().durability_volatile();

    left_marker_subscriber_ = this->create_subscription<PointMultiArray>(
        "/left/marker/data_raw", QOS_BEKL5V,
        [this](const PointMultiArray::SharedPtr msg){ left_marker_callback(msg); });

    right_marker_subscriber_ = this->create_subscription<PointMultiArray>(
        "/right/marker/data_raw", QOS_BEKL5V,
        [this](const PointMultiArray::SharedPtr msg){  right_marker_callback(msg); });
    
    matched_marker_publisher_ = this->create_publisher<PointMultiArray>(
        "/marker/data_raw", QOS_BEKL5V);

    timer_ = this->create_wall_timer(1ms, [this](){ timer_callback(); });

    left_time_ = 0.0;
    right_time_ = 0.0;

    left_markers_.reserve(3);
    right_markers_.reserve(3);
}

MarkerMatcher::~MarkerMatcher()
{
}

void MarkerMatcher::left_marker_callback(const PointMultiArray::SharedPtr msg)
{
    if (!left_markers_.empty())
        return;

    convert_msg_to_data(msg, left_time_, left_markers_);
}

void MarkerMatcher::right_marker_callback(const PointMultiArray::SharedPtr msg)
{
    if (!right_markers_.empty())
        return;
    
    convert_msg_to_data(msg, right_time_, right_markers_);
}

void MarkerMatcher::timer_callback()
{
    if ((left_markers_.empty()) || (right_markers_.empty()))
        return;

    if ((left_markers_.size() != 3) || (right_markers_.size() != 3)){
        left_markers_.clear();
        right_markers_.clear();
        return;
    }

    const double time_interval = left_time_ - right_time_;
    if (std::fabs(time_interval) < 0.005){
        std::vector<cv::Vec2f> left_pts, right_pts;

        if (left_markers_.size() <= right_markers_.size()){
            find_matching_points(left_markers_, right_markers_, 1, left_pts, right_pts);
        } else {
            find_matching_points(right_markers_, left_markers_, 2, right_pts, left_pts);
        }

        if (!left_pts.empty() && !right_pts.empty()){
            std::vector<cv::Vec2f> left_undistorted_pts, right_undistorted_pts;
            cv::undistortPoints(left_pts, left_undistorted_pts, cmtx1_, dist1_, cv::noArray(), cmtx1_);
            cv::undistortPoints(right_pts, right_undistorted_pts, cmtx2_, dist2_, cv::noArray(), cmtx2_);
            
            publish_point_msg(left_pts, right_pts);
        }

        left_markers_.clear();
        right_markers_.clear();
    } else {
        (time_interval < 0 ? left_markers_ : right_markers_).clear();
    }
}

void MarkerMatcher::convert_msg_to_data(
    const PointMultiArray::SharedPtr msg,
    double & time,
    std::vector<std::vector<double>> & markers
) {
    time = msg->stamp.sec + msg->stamp.nanosec * nano2sec;
    
    std::vector<double> data;
    data.reserve(3);
    for (const auto & p : msg->data){
        data.push_back(p.x);
        data.push_back(p.y);
        data.push_back(p.z);

        markers.push_back(data);
        data.clear();
    }
}

void MarkerMatcher::find_matching_points(
    const std::vector<std::vector<double>> & markers1,
    const std::vector<std::vector<double>> & markers2,
    const uint8_t & which_image,
    std::vector<cv::Vec2f> & output1,
    std::vector<cv::Vec2f> & output2
) {
    std::vector<cv::Vec2f> centers1;
    centers1.reserve(markers1.size());
    for (const auto & m : markers1){
        const cv::Vec2f c(m[0], m[1]);
        centers1.push_back(c);
    }

    std::vector<cv::Vec3f> epilines;
    cv::computeCorrespondEpilines(centers1, which_image, fmtx_, epilines);
   
    std::vector<std::vector<double>> error_table;
    set_error_table(markers1, markers2, epilines, which_image, error_table);
    
    std::vector<uint8_t> min_error_index;
    search_min_error_index(error_table, min_error_index);
    if (min_error_index.empty())
        return;
    
    for (uint8_t i = 0; i < min_error_index.size(); i++){
        if (error_table[i][min_error_index[i]] > 0.27)
            return;    
    }

    output1.reserve(min_error_index.size());
    output2.reserve(min_error_index.size());
    for (uint8_t i = 0; i < min_error_index.size(); i++){
        output1.push_back(centers1[i]);
        
        const cv::Vec2f c(markers2[min_error_index[i]][0], markers2[min_error_index[i]][1]);
        output2.push_back(c);
    }
}

void MarkerMatcher::set_error_table(
    const std::vector<std::vector<double>> & markers1,
    const std::vector<std::vector<double>> & markers2,
    const std::vector<cv::Vec3f> & epilines,
    const uint8_t & which_image,
    std::vector<std::vector<double>> & output
) {
    uint8_t rows = markers1.size();
    uint8_t cols = markers2.size();

    std::vector<std::vector<double>> distances(rows), losses(rows);
    double max_distance = 0.0, max_loss = 0.0;
    for (size_t i = 0; i < rows; i++){
        for (size_t j = 0; j < cols; j++){
             compute_distance(markers2[j], epilines[i], max_distance, distances[i]);
             compute_loss(markers1[i], markers2[j], which_image, max_loss, losses[i]);
        }
    }

    divide(distances, max_distance * 10. / 7.);
    divide(losses, max_loss * 10. / 3.);

    add(distances, losses, output);
}

void MarkerMatcher::compute_distance(
    const std::vector<double> & pt,
    const cv::Vec3f & line,
    double & max_value,
    std::vector<double> & output
) {
    output.push_back(std::fabs(line[0] * pt[0] + line[1] * pt[1] + line[2])
        / std::sqrt(std::pow(line[0], 2.0) + std::pow(line[1], 2.0)));

    if (max_value < output.back())
        max_value = output.back();
}

void MarkerMatcher::compute_loss(
    const std::vector<double> & pt1,
    const std::vector<double> & pt2,
    const uint8_t & which_image,
    double & max_value,
    std::vector<double> & output
) {
    double value;
    if (which_image == 1){
        inference_model(pt1, pt2, value); // left right
    } else {
        inference_model(pt2, pt1, value); // left right
    }

    output.push_back(value);
    if (max_value < value)
        max_value = value;
}

void MarkerMatcher::inference_model(
    const std::vector<double> & pt1,
    const std::vector<double> & pt2,
    double & output  
) {
    std::vector<std::vector<double>> norm_pt1, norm_pt2;
    norm_pt1.push_back(divide(pt1, maximum_value_));
    norm_pt2.push_back(divide(pt2, maximum_value_));

    std::vector<std::vector<double>> x = dot(norm_pt1, w1_);
    add(b1_, x);
    tanh(x);

    x = dot(x, w2_);    
    add(b2_, x);
    tanh(x);
    
    subtract(norm_pt2[0], x[0]);
    square(x[0]);
    output = sum(x[0]);
}

void MarkerMatcher::search_min_error_index(
    const std::vector<std::vector<double>> & error_table,
    std::vector<uint8_t> & output
) {
    const uint8_t rows = error_table.size();
    const uint8_t cols = error_table[0].size();

    output.reserve(rows);
    
    std::vector<uint8_t> init_index(cols);
    std::iota(init_index.begin(), init_index.end(), 0);

    double min_error = 1e+5;
    permutation(init_index, 0, cols, rows, error_table, min_error, output);
    
    for (size_t i = 0; i < rows; i++){
        if (error_table[i][output[i]] > 1.0){
            output.clear();
            break;
        }
    }
}

void MarkerMatcher::publish_point_msg(
    const std::vector<cv::Vec2f> & pts1,
    const std::vector<cv::Vec2f> & pts2
) {
    PointMultiArray msg;
    msg.stamp = this->get_clock()->now();
    geometry_msgs::msg::Point p;
    for (uint8_t i = 0; i < pts1.size(); i++){
        p.x = pts1[i](0);
        p.y = pts1[i](1);
        msg.data.push_back(p);
        
        p.x = pts2[i](0);
        p.y = pts2[i](1);
        msg.data.push_back(p);
    }
    matched_marker_publisher_->publish(msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    auto marker_matcher = std::make_shared<MarkerMatcher>();
    executor.add_node(marker_matcher);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
