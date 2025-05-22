#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <csignal>

using std::placeholders::_1;

cv::VideoWriter recorder;
bool is_recorder_open = false;
std::string output_file = "output.mp4";

class CamSubNode : public rclcpp::Node {
public:
    CamSubNode() : Node("camsub_stream_save") {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>( 
            "image/compressed", qos,
            std::bind(&CamSubNode::image_callback, this, _1)
        );

        // 비디오 저장을 위한 VideoWriter 객체 생성 (MJPG 코덱 사용)
        recorder.open(output_file, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, cv::Size(640, 360), true);
        if (!recorder.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video recorder!");
        } else {
            is_recorder_open = true;
            RCLCPP_INFO(this->get_logger(), "Recording started. Press Ctrl+C to stop.");
        }

        // 실시간 영상 출력을 위한 윈도우 설정
        cv::namedWindow("Live Stream", cv::WINDOW_NORMAL);
    }

    ~CamSubNode() {
        if (is_recorder_open) recorder.release();
        cv::destroyAllWindows();  // 모든 OpenCV 윈도우 종료
        RCLCPP_INFO(this->get_logger(), "Recording and display stopped.");
    }

private:
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        // 메시지에서 이미지를 디코딩
        cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        if (frame.empty()) return;

        // 실시간 영상 출력
        cv::imshow("Live Stream", frame);

        // 영상이 유효하면 기록
        if (is_recorder_open) recorder.write(frame);

        // 1ms 대기하여 화면을 갱신 (esc 키를 누르면 종료)
        cv::waitKey(1);
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
