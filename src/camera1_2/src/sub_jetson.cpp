#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"
#include <memory>
#include <functional>
#include <csignal>

using std::placeholders::_1;

cv::VideoWriter streamer;
cv::VideoWriter recorder;
bool is_recorder_open = false;
bool is_streamer_open = false;
std::string output_file = "output.mp4";

class CamSubNode : public rclcpp::Node {
public:
    CamSubNode() : Node("camsub_stream_save") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "image/compressed", qos,
            std::bind(&CamSubNode::image_callback, this, _1)
        );

        std::string gstreamer_pipeline =
            "appsrc ! videoconvert ! video/x-raw, format=BGRx ! "
            "nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! "
            "h264parse ! rtph264pay pt=96 ! "
            "udpsink host=203.234.58.169 port=8002 sync=false";

        streamer.open(gstreamer_pipeline, 0, 30.0, cv::Size(640, 360), true);
        if (!streamer.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open GStreamer streamer!");
        } else {
            is_streamer_open = true;
        }

        recorder.open(output_file, cv::VideoWriter::fourcc('a','v','c','1'), 30.0, cv::Size(640, 360), true);
        if (!recorder.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video recorder!");
        } else {
            is_recorder_open = true;
            RCLCPP_INFO(this->get_logger(), "Recording started. Press Ctrl+C to stop.");
        }
    }

    ~CamSubNode() {
        if (is_recorder_open) recorder.release();
        if (is_streamer_open) streamer.release();
        RCLCPP_INFO(this->get_logger(), "Recording and streaming stopped.");
    }

private:
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (frame.empty()) return;

        if (is_streamer_open) streamer.write(frame);
        if (is_recorder_open) recorder.write(frame);
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CamSubNode>();

    std::signal(SIGINT, [](int) {
        rclcpp::shutdown();
    });

    rclcpp::spin(node);
    return 0;
}