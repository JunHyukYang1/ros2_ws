#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher;
cv::VideoCapture cap;
std::string video_path;

void timer_callback(rclcpp::Node::SharedPtr node)
{
  cv::Mat frame;
  if (!cap.read(frame))
  {
    RCLCPP_INFO(node->get_logger(), "Video finished. Looping...");
    cap.set(cv::CAP_PROP_POS_FRAMES, 0);
    return;
  }

  cv::Mat resized;
  cv::resize(frame, resized, cv::Size(640, 480));

  std_msgs::msg::Header header;
  header.stamp = node->now();
  header.frame_id = "camera";

  cv_bridge::CvImage cv_image(header, "bgr8", resized);
  sensor_msgs::msg::CompressedImage msg;
  msg.header = header;
  msg.format = "jpeg";
  cv::imencode(".jpg", resized, msg.data);

  publisher->publish(msg);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("video_publisher_node");

  node->declare_parameter<std::string>("video_path", "/home/linux/ros2_ws/src/video_publisher/lidar_scan.mp4");
  node->get_parameter("video_path", video_path);

  cap.open(video_path);
  if (!cap.isOpened())
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to open video file: %s", video_path.c_str());
    return -1;
  }

  publisher = node->create_publisher<sensor_msgs::msg::CompressedImage>("/image/compressed", 10);

  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(33),
    [node]() { timer_callback(node); }
  );

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}