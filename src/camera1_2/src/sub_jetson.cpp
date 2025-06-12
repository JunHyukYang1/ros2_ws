#include "rclcpp/rclcpp.hpp"                               // ROS 2의 기본 노드 및 로깅 기능 포함
#include "sensor_msgs/msg/compressed_image.hpp"            // 압축된 이미지 메시지 타입
#include "opencv2/opencv.hpp"                              // OpenCV 기본 헤더
#include <memory>                                          // 메모리 관리, 함수 객체 사용
#include <functional>
#include <iostream>

using std::placeholders::_1;  // std::bind에서 자리 표시자(_1) 사용

// GStreamer 파이프라인: 비디오를 RTP 형식으로 UDP 전송
std::string dst = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
    nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
    h264parse ! rtph264pay pt=96 ! \
    udpsink host=192.168.0.13 port=8001 sync=false";

// GStreamer를 이용한 UDP 스트리밍용 VideoWriter
cv::VideoWriter writer;

// MP4 파일로 저장하기 위한 VideoWriter
cv::VideoWriter recorder;

// 콜백 함수: 구독한 이미지 메시지를 처리하는 함수
void mysub_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // 압축된 이미지 데이터를 OpenCV Mat으로 디코딩
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (frame.empty()) {
        // 디코딩 실패 시 경고 출력
        RCLCPP_WARN(node->get_logger(), "Empty frame received!");
        return;
    }

    // UDP로 실시간 스트리밍
    writer.write(frame);

    // 로컬 파일로 녹화
    recorder.write(frame);

    // 수신 로그 출력
    RCLCPP_INFO(node->get_logger(), "Received Image: %s, %d x %d",
                msg->format.c_str(), frame.rows, frame.cols);
}

// main 함수: 노드 초기화 및 구독 설정
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);  // ROS 2 초기화

    // 노드 생성
    auto node = std::make_shared<rclcpp::Node>("camsub");

    // GStreamer 파이프라인으로 VideoWriter 열기 (스트리밍용)
    writer.open(dst, 0, 30.0, cv::Size(640, 360), true);
    if (!writer.isOpened()) {
        // 열기 실패 시 오류 출력 및 종료
        RCLCPP_ERROR(node->get_logger(), "Failed to open GStreamer writer!");
        rclcpp::shutdown();
        return -1;
    }

    // MP4 저장용 파일 이름
    std::string output_file = "output.mp4";

    // MP4 녹화용 VideoWriter 열기 (H.264 인코딩, 30fps, 640x360)
    recorder.open(output_file, cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 30.0, cv::Size(640, 360), true);
    if (!recorder.isOpened()) {
        // 열기 실패 시 오류 출력 및 종료
        RCLCPP_ERROR(node->get_logger(), "Failed to open MP4 recorder!");
        rclcpp::shutdown();
        return -1;
    }

    // QoS 설정: Best Effort, 큐 길이 10
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // 콜백 함수 연결(bind)
    auto fn = std::bind(mysub_callback, node, _1);

    // 압축 이미지 구독 설정
    auto mysub = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos_profile, fn);

    // 콜백 함수 이벤트 루프 실행
    rclcpp::spin(node);

    // 종료 시 VideoWriter 해제
    writer.release();
    recorder.release();

    // ROS 2 종료
    rclcpp::shutdown();
    return 0;
}
