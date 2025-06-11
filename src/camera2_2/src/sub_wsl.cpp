#include "rclcpp/rclcpp.hpp"  // ROS 2 C++ 클라이언트 라이브러리
#include "sensor_msgs/msg/compressed_image.hpp"  // CompressedImage 메시지 타입
#include "opencv2/opencv.hpp"  // OpenCV 기능 사용
#include <memory>  // std::shared_ptr 사용
#include <functional>  // std::bind, std::function 등 사용
#include <csignal>  // 시그널 핸들링 (Ctrl+C 종료)

using std::placeholders::_1;  // std::bind에서 자리 표시자 _1 사용

// 전역 VideoWriter 객체 (녹화용)
cv::VideoWriter recorder;
// 녹화기 오픈 여부 확인 변수
bool is_recorder_open = false;
// 저장할 비디오 파일 경로
std::string output_file = "output.mp4";

// 이미지 수신 콜백 함수
void image_callback(rclcpp::Node::SharedPtr node, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // 메시지의 압축된 데이터를 OpenCV 이미지로 디코딩
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

    // 디코딩 실패 시 무시
    if (frame.empty()) return;

    // 실시간 영상 표시
    cv::imshow("Live Stream", frame);

    // 녹화기가 열려 있다면 프레임 저장
    if (is_recorder_open)
        recorder.write(frame);

    // OpenCV 창 갱신 (1ms 대기)
    cv::waitKey(1);
}

int main(int argc, char* argv[])
{
    // ROS 2 초기화
    rclcpp::init(argc, argv);

    // 노드 생성
    auto node = std::make_shared<rclcpp::Node>("camsub_stream_save");

    // QoS 설정: 최근 10개 메시지를 저장하고 Best Effort로 수신
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

    // 압축 이미지 수신 콜백 함수 바인딩 (노드 포함)
    std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr)> cb =
        std::bind(image_callback, node, _1);

    // "image/compressed" 토픽 구독자 생성
    auto subscription = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", qos, cb);

    // 비디오 저장을 위한 VideoWriter 초기화 (MJPG 코덱, 640x360, 30FPS)
    recorder.open(output_file, cv::VideoWriter::fourcc('M','J','P','G'), 30.0, cv::Size(640, 360), true);

    // 열기에 실패한 경우 로그 출력
    if (!recorder.isOpened()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open video recorder!");
    } else {
        // 정상 열리면 상태 저장 및 로그 출력
        is_recorder_open = true;
        RCLCPP_INFO(node->get_logger(), "Recording started. Press Ctrl+C to stop.");
    }

    // 실시간 영상을 위한 OpenCV 윈도우 생성
    cv::namedWindow("Live Stream", cv::WINDOW_NORMAL);

    // Ctrl+C를 누르면 ROS 종료되도록 시그널 핸들러 등록
    std::signal(SIGINT, [](int){
        rclcpp::shutdown();
    });

    // ROS 메시지 수신 루프 시작
    rclcpp::spin(node);

    // 종료 시 리소스 정리
    if (is_recorder_open)
        recorder.release();  // 녹화기 해제

    cv::destroyAllWindows();  // OpenCV 윈도우 닫기

    RCLCPP_INFO(node->get_logger(), "Recording and display stopped.");

    return 0;  // 프로그램 종료
}
