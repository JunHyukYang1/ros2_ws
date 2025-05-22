#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
using std::placeholders::_1;

void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // 압축된 이미지를 cv::Mat으로 디코딩
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data),  cv::IMREAD_COLOR);
    
    // 그레이스케일 영상으로 변환
    cv::Mat gray_frame;
    cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
    
    // 이진영상으로 변환 (임계값 128 사용)
    cv::Mat binary_frame;
    cv::threshold(gray_frame, binary_frame, 128, 255, cv::THRESH_BINARY);

    // 원본, 그레이스케일, 이진영상을 각각 윈도우에 표시
    cv::imshow("Original Image", frame);
    cv::imshow("Grayscale Image", gray_frame);
    cv::imshow("Binary Image", binary_frame);
    
    cv::waitKey(1);  // OpenCV의 waitKey로 윈도우가 업데이트되도록 함
    
    // 로그로 영상 정보 출력
    RCLCPP_INFO(node->get_logger(), "Received Image: Format: %s, Size: (%d, %d)", 
                 msg->format.c_str(), frame.rows, frame.cols);
}

int main(int argc, char* argv[])
{
    // ROS 2 초기화
    rclcpp::init(argc, argv);

    // 노드 생성
    auto node = std::make_shared<rclcpp::Node>("camsub_wsl");

    // QoS 설정 (KeepLast 10, Best Effort)
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // 콜백 함수 설정
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);
    
    // "image/compressed" 토픽에 구독자 생성
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile, fn);

    // ROS 2 spin을 통해 메시지 수신 대기
    rclcpp::spin(node);
    
    // 종료 시 ROS 2 종료
    rclcpp::shutdown();
    
    return 0;
}
