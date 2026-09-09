#include "rclcpp/rclcpp.hpp"
#include "shm/SharedImage.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <time.h>

class ZeroCopyForwarder : public rclcpp::Node {
  public:
    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    // Shared Image Writer
    std::unique_ptr<SharedImageWriter> sharedImageWriter_;

    struct timespec curr_ts {};

    ZeroCopyForwarder() : Node("image_publisher") {
        // ROS Queue and Publisher initialization
        rclcpp::QoS custom_qos(1);
        custom_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        custom_qos.durability(rclcpp::DurabilityPolicy::Volatile);
        custom_qos.history(rclcpp::HistoryPolicy::KeepLast);
        this->subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image", custom_qos,
            std::bind(&ZeroCopyForwarder::forwardCallback, this, std::placeholders::_1));
        this->sharedImageWriter_ =
            std::make_unique<SharedImageWriter>("camera_image", 1200, 1920, 3, "CV_8U");
    }

    void forwardImage(const cv::Mat &img, rclcpp::Time grabbedTimestamp) {
        // Code borrowed from tr-camera-basler/CMyImageHandler.cpp
        int64_t total_ns = grabbedTimestamp.nanoseconds();
        curr_ts.tv_sec = total_ns / 1'000'000'000L;
        curr_ts.tv_nsec = total_ns % 1'000'000'000L;

        if (!sharedImageWriter_->writeImage(img, curr_ts)) {
            RCLCPP_INFO(this->get_logger(), "Write to shared memory failed.");
        }
    }

    void forwardCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        auto img = cv_bridge::toCvShare(msg, "bgr8");
        rclcpp::Time grabbedTimestamp(msg->header.stamp);
        forwardImage(img->image, grabbedTimestamp);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ZeroCopyForwarder>());
    rclcpp::shutdown();
    return 0;
}