#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>

class LineErrorNode : public rclcpp::Node
{
public:
    LineErrorNode()
    : Node("line_error_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
            "/image/compressed", 10,
            std::bind(&LineErrorNode::image_callback, this, std::placeholders::_1));

        cv::namedWindow("Original Video", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Labeled ROI", cv::WINDOW_AUTOSIZE);
    }

    ~LineErrorNode()
    {
        cv::destroyAllWindows();
    }

private:
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        // 압축 이미지 디코딩
        cv::Mat frame = cv::imdecode(msg->data, cv::IMREAD_COLOR);
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Failed to decode image");
            return;
        }

        // 그레이스케일 + 블러
        cv::Mat gray, binary;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(5, 5), 0);

        // 이진화: 흰색 라인
        cv::threshold(gray, binary, 150, 255, cv::THRESH_BINARY);

        // ROI: 하단 30%
        int height = binary.rows;
        int width = binary.cols;
        int roi_start_y = static_cast<int>(height * 0.7);
        cv::Mat roi = binary(cv::Range(roi_start_y, height), cv::Range::all());

        // 연결 요소 분석
        cv::Mat labels, stats, centroids;
        int n_labels = cv::connectedComponentsWithStats(roi, labels, stats, centroids);

        // 가장 큰 라벨 찾기 (0은 배경)
        int max_area = 0;
        int target_label = -1;
        for (int i = 1; i < n_labels; ++i) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area > max_area) {
                max_area = area;
                target_label = i;
            }
        }

        cv::imshow("Original Video", frame);

        if (target_label != -1) {
            double cx = centroids.at<double>(target_label, 0);
            double error = cx - (width / 2.0);
            RCLCPP_INFO(this->get_logger(), "Error from labeled centroid: %.2f", error);

            // 시각화용 컬러 ROI
            cv::Mat roi_color;
            cv::cvtColor(roi, roi_color, cv::COLOR_GRAY2BGR);
            cv::circle(roi_color, cv::Point(static_cast<int>(cx), roi.rows / 2), 7, cv::Scalar(0, 255, 255), -1);
            cv::imshow("Labeled ROI", roi_color);
        } else {
            RCLCPP_WARN(this->get_logger(), "No valid line label found.");
        }

        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LineErrorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
