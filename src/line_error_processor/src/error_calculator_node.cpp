#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/int32.hpp"
#include "opencv2/opencv.hpp"

using std::placeholders::_1;

class ErrorCalculatorNode : public rclcpp::Node
{
public:
    ErrorCalculatorNode() : Node("error_calculator_node")
    {
        // 퍼블리시 노드 토픽명에 맞춰 구독 토픽명 변경
        image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "image/compressed", 10,
            std::bind(&ErrorCalculatorNode::image_callback, this, _1));

        error_pub_ = this->create_publisher<std_msgs::msg::Int32>("line_tracer/error", 10);

        RCLCPP_INFO(this->get_logger(), "라인 에러 계산 노드 시작됨");
    }

private:
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        try
        {
            // 압축된 이미지 데이터를 OpenCV Mat로 디코딩
            cv::Mat image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

            if (image.empty())
                return;

            // 관심영역(ROI): 영상 하단 1/4 부분
            cv::Rect roi(0, image.rows * 3 / 4, image.cols, image.rows / 4);
            cv::Mat cropped = image(roi);

            // 그레이스케일 변환 및 이진화 (선 검출)
            cv::Mat gray, binary;
            cv::cvtColor(cropped, gray, cv::COLOR_BGR2GRAY);
            cv::threshold(gray, binary, 100, 255, cv::THRESH_BINARY_INV);

            // 무게중심 계산 (선의 중심 좌표)
            cv::Moments m = cv::moments(binary, true);
            int error = 0;

            if (m.m00 > 0)
            {
                int cx = static_cast<int>(m.m10 / m.m00);
                int center = binary.cols / 2;
                error = cx - center;
            }

            std_msgs::msg::Int32 error_msg;
            error_msg.data = error;
            error_pub_->publish(error_msg);

        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV 예외: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr error_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ErrorCalculatorNode>());
    rclcpp::shutdown();
    return 0;
}
