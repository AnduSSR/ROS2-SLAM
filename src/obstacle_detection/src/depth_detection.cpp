#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

using std::placeholders::_1;
using std::placeholders::_2;

// Stereo Camera Parameters (Replace with your actual calibration data)
const float FOCAL_LENGTH = 700.0f; // in pixels
const float BASELINE = 0.12f;      // distance between cameras in meters

class StereoDepthEstimator : public rclcpp::Node {
public:
    StereoDepthEstimator() : Node("stereo_depth_estimator") {
        left_sub_.subscribe(this, "/camera/left/image_raw");
        right_sub_.subscribe(this, "/camera/right/image_raw");

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), left_sub_, right_sub_);
        
        sync_->registerCallback(std::bind(&StereoDepthEstimator::stereo_callback, this, _1, _2));

        // Setup OpenCV Stereo SGBM algorithm
        stereo_matcher_ = cv::StereoSGBM::create(
            0,    // minDisparity
            160,  // numDisparities (must be divisible by 16)
            11,   // blockSize
            8 * 11 * 11,  // P1
            32 * 11 * 11, // P2
            1,    // disp12MaxDiff
            63,   // preFilterCap
            10,   // uniquenessRatio
            100,  // speckleWindowSize
            32,   // speckleRange
            cv::StereoSGBM::MODE_SGBM
        );

        RCLCPP_INFO(this->get_logger(), "Stereo depth node initialized and waiting for images...");
    }

private:
    // Define the synchronization policy type
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    
    message_filters::Subscriber<sensor_msgs::msg::Image> left_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    cv::Ptr<cv::StereoSGBM> stereo_matcher_;

    void stereo_callback(const sensor_msgs::msg::Image::ConstSharedPtr& left_msg, 
                         const sensor_msgs::msg::Image::ConstSharedPtr& right_msg) {
        try {
            // Convert ROS images to OpenCV matrices (Grayscale)
            cv::Mat left_img = cv_bridge::toCvCopy(left_msg, "mono8")->image;
            cv::Mat right_img = cv_bridge::toCvCopy(right_msg, "mono8")->image;

            cv::Mat disparity_16s, disparity_float;

            // Compute disparity map
            stereo_matcher_->compute(left_img, right_img, disparity_16s);

            // OpenCV SGBM returns disparities multiplied by 16. We must divide by 16.0 to get true pixel disparity.
            disparity_16s.convertTo(disparity_float, CV_32F, 1.0 / 16.0);

            // Extract depth at the center pixel
            int u = left_img.cols / 2;
            int v = left_img.rows / 2;
            
            float pixel_disparity = disparity_float.at<float>(v, u);

            if (pixel_disparity > 0.0f) {
                // Apply Z = (f * b) / d
                float depth_z = (FOCAL_LENGTH * BASELINE) / pixel_disparity;

                RCLCPP_INFO(this->get_logger(), "Center Disparity: %.2f pixels | Estimated Depth: %.3f meters", 
                            pixel_disparity, depth_z);
            } else {
                RCLCPP_WARN(this->get_logger(), "Unable to compute valid depth at center (no disparity matched).");
            }

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoDepthEstimator>());
    rclcpp::shutdown();
    return 0;
}