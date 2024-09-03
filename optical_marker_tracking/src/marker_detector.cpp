#include "optical_marker_tracking/marker_detector.hpp"
#include "optical_tracking_msgs/optical_tracking_utils.hpp"

#include <algorithm>

MarkerDetector::MarkerDetector() : Node("marker_detector")
{
    const auto QOS_BEKL5V = rclcpp::QoS(rclcpp::KeepLast(5))
        .best_effort().durability_volatile();

    image_subscriber_ = this->create_subscription<Image>("camera/data_raw", QOS_BEKL5V,
        [this](const Image::SharedPtr msg){ image_callback(msg); });

    marker_publisher_ = this->create_publisher<PointMultiArray>(
        "marker/data_raw", QOS_BEKL5V);

    cv::setNumThreads(4);
}

MarkerDetector::~MarkerDetector()
{
}

void MarkerDetector::image_callback(const Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch (const cv_bridge::Exception & e)
    {
        RCLCPP_ERROR(this->get_logger(), 
            "[MarkerDetector::image_callback] cv_bridge exception: %s", e.what());
        return;
    }
    const cv::Mat & frame = cv_ptr->image;

    std::vector<cv::Vec3f> pts;
    pts.reserve(4);
    segment_markers(frame, pts);
    
    if (pts.empty())
        return;

    publish_point_msg(pts);
}

void MarkerDetector::publish_point_msg(const std::vector<cv::Vec3f> & pts)
{
    PointMultiArray msg;
    msg.stamp = this->get_clock()->now();
    optical_tracking_msgs::convert_vector_to_msg_3d(pts, msg.data);
    marker_publisher_->publish(msg);
}

void MarkerDetector::segment_markers(const cv::Mat & frame, std::vector<cv::Vec3f> & pts)
{
    cv::Mat blured, ycrcb[3];
    const cv::Size kernel_size(0, 0);
    const double gaussian_sigma = 1.0;

    cv::GaussianBlur(frame, blured, kernel_size, gaussian_sigma);
    cv::cvtColor(blured, blured, cv::COLOR_BGR2YCrCb);
    cv::split(blured, ycrcb);

    cv::threshold(ycrcb[0], ycrcb[0], 70, 255, cv::THRESH_TOZERO);
    cv::threshold(ycrcb[0], ycrcb[0], 200, 255, cv::THRESH_TOZERO_INV);
    cv::threshold(ycrcb[0], ycrcb[0], 1, 255, cv::THRESH_BINARY);
    
    cv::threshold(ycrcb[1], ycrcb[1], 170, 255, cv::THRESH_TOZERO);
    cv::threshold(ycrcb[1], ycrcb[1], 220, 255, cv::THRESH_TOZERO_INV);
    cv::threshold(ycrcb[1], ycrcb[1], 1, 255, cv::THRESH_BINARY);
    
    cv::threshold(ycrcb[2], ycrcb[2], 140, 255, cv::THRESH_TOZERO_INV);
    cv::threshold(ycrcb[2], ycrcb[2], 95, 255, cv::THRESH_TOZERO);
    cv::threshold(ycrcb[2], ycrcb[2], 1, 255, cv::THRESH_BINARY);
    
    const cv::Mat mask = ycrcb[0] & ycrcb[1] & ycrcb[2];
    
    std::vector<cv::Vec3f> circles;
    
    cv::HoughCircles(mask, circles, cv::HOUGH_GRADIENT, 1, 10, 50, 8, 5, 15);

    if (circles.empty())
        return;

    std::vector<std::vector<cv::Point>> cnts;
    for (const auto & c : circles){
        const int x_min = std::max(static_cast<int>(c[0] - c[2]), 0);
        const int y_min = std::max(static_cast<int>(c[1] - c[2]), 0);

        const int x_max = std::min(static_cast<int>(c[0] + c[2]), 1280);
        const int y_max = std::min(static_cast<int>(c[1] + c[2]), 720);

        cnts.clear();
        cv::findContours(mask(cv::Rect(cv::Point2i(x_min, y_min), cv::Point2i(x_max, y_max))),
                         cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        if (cnts.empty())
            continue;
        
        double largest_area = 0.0;
        for (const auto & cnt : cnts){
            double area = cv::contourArea(cnt, false);
            if (largest_area < area)
                largest_area = area;
        }
        
        if (largest_area > 20)
            pts.push_back(c);
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto marker_detector = std::make_shared<MarkerDetector>();
    rclcpp::spin(marker_detector);
    rclcpp::shutdown();

    return 0;
}
