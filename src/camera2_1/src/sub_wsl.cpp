#include "rclcpp/rclcpp.hpp"  // ROS 2 C++ 클라이언트 라이브러리
#include "sensor_msgs/msg/compressed_image.hpp"  // 압축 이미지 메시지 타입
#include "opencv2/opencv.hpp"  // OpenCV 주요 기능 포함 헤더
#include <memory>  // std::shared_ptr 등 스마트 포인터 사용을 위해
#include <functional>  // std::bind 및 std::function 사용을 위해
#include <iostream>  // 표준 출력 사용을 위해

using std::placeholders::_1;  // std::bind에서 자리 표시자로 사용되는 _1을 이름 없이 사용

// 이미지 수신 콜백 함수 정의
void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // 압축된 이미지를 OpenCV 형식(cv::Mat)으로 디코딩 (BGR 컬러 이미지로 복원)
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    
    // 그레이스케일 영상으로 변환
    cv::Mat gray_frame;
    cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);
    
    // 이진 영상으로 변환 (임계값 128, 최대값 255 사용)
    cv::Mat binary_frame;
    cv::threshold(gray_frame, binary_frame, 128, 255, cv::THRESH_BINARY);

    // 각 이미지를 OpenCV 창에 표시
    cv::imshow("Original Image", frame);        // 원본 영상
    cv::imshow("Grayscale Image", gray_frame);  // 그레이 영상
    cv::imshow("Binary Image", binary_frame);   // 이진 영상
    
    cv::waitKey(1);  // 화면 갱신 및 이벤트 처리를 위해 짧게 대기 (1ms)

    // ROS 로그로 이미지 형식 및 크기 정보 출력
    RCLCPP_INFO(node->get_logger(), "Received Image: Format: %s, Size: (%d, %d)", 
                 msg->format.c_str(), frame.rows, frame.cols);
}

// 메인 함수 시작
int main(int argc, char* argv[])
{
    // ROS 2 초기화
    rclcpp::init(argc, argv);

    // 노드 생성 (노드 이름: "camsub_wsl")
    auto node = std::make_shared<rclcpp::Node>("camsub_wsl");

    // QoS 설정: 최근 10개 메시지 저장, Best Effort 신뢰도 (이미지 송신자와 호환되도록 설정)
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // 콜백 함수 등록 (mysub_callback에 노드를 바인딩하여 std::function으로 전달)
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);
    
    // "image/compressed" 토픽 구독 설정 (압축 이미지 수신)
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile, fn);

    // 메시지를 수신하며 콜백을 실행하기 위해 spin 루프 시작
    rclcpp::spin(node);
    
    // 종료되면 ROS 2 종료
    rclcpp::shutdown();
    
    return 0;  // 프로그램 종료
}
