#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>

class VideoPublisher : public rclcpp::Node
{
public:
    VideoPublisher()
    : Node("video_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/image/compressed", 10);

        // 영상 파일 경로 수정 필요
        cap_.open("/home/linux/test.mp4");
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file");
            throw std::runtime_error("Failed to open video file");
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),  // 약 30 FPS
            std::bind(&VideoPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        cv::Mat frame;
        if (!cap_.read(frame)) {
            RCLCPP_INFO(this->get_logger(), "Video ended, restarting...");
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
            return;
        }

        std::vector<uchar> buf;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
        if (!cv::imencode(".jpg", frame, buf, params)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to encode frame");
            return;
        }

        auto msg = sensor_msgs::msg::CompressedImage();
        msg.header.stamp = this->get_clock()->now();
        msg.format = "jpeg";
        msg.data.assign(buf.begin(), buf.end());

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Published one frame");
    }

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<VideoPublisher>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
