#ifndef MARKER_MATCHER_HPP_
#define MARKER_MATCHER_HPP_

#include <memory>
#include <vector>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "optical_tracking_msgs/msg/point_multi_array.hpp"

#include <opencv2/opencv.hpp>

class MarkerMatcher : public rclcpp::Node
{
public:
    using PointMultiArray = optical_tracking_msgs::msg::PointMultiArray;

    MarkerMatcher();
    virtual ~MarkerMatcher();

private:
    void convert_msg_to_data(
        const PointMultiArray::SharedPtr msg,
        double & time,
        std::vector<std::vector<double>> & markers
    );

    void find_matching_points(
        const std::vector<std::vector<double>> & markers1,
        const std::vector<std::vector<double>> & markers2,
        const uint8_t & which_image,       
        std::vector<cv::Vec2f> & output1,
        std::vector<cv::Vec2f> & output2
    );
    
    void set_error_table(
        const std::vector<std::vector<double>> & markers1,
        const std::vector<std::vector<double>> & markers2,
        const std::vector<cv::Vec3f> & epilines,
        const uint8_t & which_image,
        std::vector<std::vector<double>> & output    
    );

    void compute_distance(
        const std::vector<double> & pts,
        const cv::Vec3f & lines,
        double & max_value,
        std::vector<double> & output
    );

    void compute_loss(
        const std::vector<double> & pt1,
        const std::vector<double> & pt2,
        const uint8_t & which_image,
        double & max_value,
        std::vector<double> & output
    );
    
    void inference_model(
        const std::vector<double> & pt1,
        const std::vector<double> & pt2,
        double & output
    );

    void search_min_error_index(
        const std::vector<std::vector<double>> & error_table,
        std::vector<uint8_t> & output        
    );

    void publish_point_msg(
        const std::vector<cv::Vec2f> & pts1,
        const std::vector<cv::Vec2f> & pts2        
    );

    void left_marker_callback(const PointMultiArray::SharedPtr msg);
    void right_marker_callback(const PointMultiArray::SharedPtr msg);
    void timer_callback();

    rclcpp::Subscription<PointMultiArray>::SharedPtr left_marker_subscriber_;
    rclcpp::Subscription<PointMultiArray>::SharedPtr right_marker_subscriber_;
    rclcpp::Publisher<PointMultiArray>::SharedPtr matched_marker_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    const cv::Mat cmtx1_, dist1_, cmtx2_, dist2_, fmtx_;
    const std::vector<std::vector<double>> w1_, b1_, w2_, b2_;
    const std::vector<double> maximum_value_;

    std::vector<std::vector<double>> left_markers_, right_markers_;
    double left_time_, right_time_;
};

#endif
