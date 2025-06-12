#include "rclcpp/rclcpp.hpp"                           // ROS 2 C++ 클라이언트 라이브러리 포함
#include "sensor_msgs/msg/compressed_image.hpp"        // CompressedImage 메시지 타입 포함
#include "opencv2/opencv.hpp"                          // OpenCV 라이브러리 포함
#include <memory>                                      // 스마트 포인터 사용을 위한 헤더
#include <functional>                                  // std::bind 등을 위한 헤더
#include <iostream>                                    // 입출력 스트림
#include <vector>                                      // std::vector 사용을 위한 헤더

using std::placeholders::_1;                           // std::placeholders::_1를 네임스페이스로 사용

// GStreamer 파이프라인 문자열 정의 (UDP를 통해 인코딩된 비디오를 외부 IP로 전송)
std::string dst = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
    nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
    h264parse ! rtph264pay pt=96 ! \
    udpsink host=192.168.0.13 port=8001 sync=false";

// 전역 VideoWriter 객체 생성
cv::VideoWriter writer;

// CompressedImage 콜백 함수 정의
void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // 압축된 이미지를 OpenCV의 Mat 형식으로 디코딩
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    
    // 디코딩 실패 시 에러 출력 및 반환
    if (frame.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to decode image");
        return;
    }
    
    // 디코딩된 컬러 이미지를 그레이스케일로 변환
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    // 이진화 (OTSU 알고리즘 사용) 수행
    cv::Mat binary;
    cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // 이진화된 영상을 GStreamer 파이프라인으로 송출
    writer << binary;
    
    // 디버깅을 위한 로그 출력
    RCLCPP_INFO(node->get_logger(), "Received Image: %s, %d x %d, Binary Image Size: %d x %d", 
                 msg->format.c_str(), frame.cols, frame.rows, binary.cols, binary.rows);
}

int main(int argc, char * argv[])
{
    // ROS 2 노드 초기화
    rclcpp::init(argc, argv);

    // 노드 생성
    auto node = std::make_shared<rclcpp::Node>("camera1_1_node");

    // VideoWriter 객체를 GStreamer 파이프라인과 연결 (흑백 영상이므로 마지막 인자는 false)
    writer.open(dst, 0, (double)30, cv::Size(640, 360), false);

    // writer가 정상적으로 열리지 않으면 에러 출력 후 종료
    if (!writer.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Writer open failed!");
        rclcpp::shutdown();
        return -1;
    }

    // QoS 설정: 최신 메시지 10개까지 저장, best_effort 모드 (실시간 성능 위주)
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // 콜백 함수 바인딩 (node를 캡처하여 콜백 내부에서 로그 출력 등 가능하게 함)
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
    fn = std::bind(mysub_callback, node, _1);

    // 이미지 구독자 생성 (image/compressed 토픽 구독)
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos_profile, fn);

    // 노드 실행 (콜백 대기)
    rclcpp::spin(node);

    // 노드 종료
    rclcpp::shutdown();
    return 0;
}
