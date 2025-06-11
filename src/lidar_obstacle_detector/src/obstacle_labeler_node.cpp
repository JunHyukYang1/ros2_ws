#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <cmath>

using std::placeholders::_1;

rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription;

double calculateAngle(const cv::Point& origin, const cv::Point& pt) {
    double dx = pt.x - origin.x;
    double dy = origin.y - pt.y;  // 화면 좌표계 y 반전 주의
    double angle = std::atan2(dy, dx) * 180.0 / CV_PI;
    if (angle < 0) angle += 360;
    return angle;
}

cv::Point calcIntersectionWithBBox(const cv::Point& center, const cv::Rect& bbox) {
    double dx = (bbox.x + bbox.width / 2.0) - center.x;
    double dy = (bbox.y + bbox.height / 2.0) - center.y;

    double left = bbox.x;
    double right = bbox.x + bbox.width;
    double top = bbox.y;
    double bottom = bbox.y + bbox.height;

    std::vector<cv::Point2d> candidates;

    if (dx != 0) {
        double t1 = (left - center.x) / dx;
        double y1 = center.y + t1 * dy;
        if (t1 > 0 && y1 >= top && y1 <= bottom) {
            candidates.emplace_back(left, y1);
        }
        double t2 = (right - center.x) / dx;
        double y2 = center.y + t2 * dy;
        if (t2 > 0 && y2 >= top && y2 <= bottom) {
            candidates.emplace_back(right, y2);
        }
    }

    if (dy != 0) {
        double t3 = (top - center.y) / dy;
        double x3 = center.x + t3 * dx;
        if (t3 > 0 && x3 >= left && x3 <= right) {
            candidates.emplace_back(x3, top);
        }
        double t4 = (bottom - center.y) / dy;
        double x4 = center.x + t4 * dx;
        if (t4 > 0 && x4 >= left && x4 <= right) {
            candidates.emplace_back(x4, bottom);
        }
    }

    double min_dist = 1e9;
    cv::Point2d nearest_point;
    for (auto& p : candidates) {
        double dist = cv::norm(cv::Point2d(center) - p);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_point = p;
        }
    }

    if (candidates.empty()) {
        return cv::Point(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);
    }

    return cv::Point(static_cast<int>(nearest_point.x), static_cast<int>(nearest_point.y));
}

cv::Point calcPointFromAngleLength(const cv::Point& origin, double angle_deg, double length) {
    double angle_rad = angle_deg * CV_PI / 180.0;
    int x = static_cast<int>(origin.x + length * cos(angle_rad));
    int y = static_cast<int>(origin.y - length * sin(angle_rad));
    return cv::Point(x, y);
}

void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat img = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (img.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "빈 이미지입니다.");
        return;
    }

    cv::resize(img, img, cv::Size(500, 500));
    cv::Rect roi(0, 0, 500, 250);
    cv::Mat roi_img = img(roi);

    cv::Mat gray;
    cv::cvtColor(roi_img, gray, cv::COLOR_BGR2GRAY);

    cv::Mat thresh;
    cv::adaptiveThreshold(gray, thresh, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                          cv::THRESH_BINARY_INV, 15, 10);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresh, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Point center(250, 250);

    double min_dist_red = 1e9;
    double min_dist_blue = 1e9;
    cv::Rect nearest_red_bbox;
    cv::Rect nearest_blue_bbox;
    bool red_found = false;
    bool blue_found = false;

    // 가장 가까운 빨간/파란 장애물만 찾기
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area < 100) continue;

        cv::Rect bbox = cv::boundingRect(contours[i]);
        cv::Point bbox_center(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);

        bool is_red = bbox_center.x < 250;

        double dist = cv::norm(center - bbox_center);

        if (is_red && dist < min_dist_red) {
            min_dist_red = dist;
            nearest_red_bbox = bbox;
            red_found = true;
        }

        if (!is_red && dist < min_dist_blue) {
            min_dist_blue = dist;
            nearest_blue_bbox = bbox;
            blue_found = true;
        }
    }

    // 가장 가까운 빨간 장애물에만 바운딩 박스와 라벨링
    if (red_found) {
        cv::rectangle(img, nearest_red_bbox, cv::Scalar(0, 0, 255), 2);
        std::string label = "Red Obj";
        cv::putText(img, label, cv::Point(nearest_red_bbox.x, nearest_red_bbox.y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
    }

    // 가장 가까운 파란 장애물에만 바운딩 박스와 라벨링
    if (blue_found) {
        cv::rectangle(img, nearest_blue_bbox, cv::Scalar(255, 0, 0), 2);
        std::string label = "Blue Obj";
        cv::putText(img, label, cv::Point(nearest_blue_bbox.x, nearest_blue_bbox.y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
    }

    auto drawArrow = [&](const cv::Point& start, const cv::Point& end, const cv::Scalar& color) {
        cv::arrowedLine(img, start, end, color, 2);
    };

    // 빨간 화살표
    cv::Point red_point;
    if (red_found) {
        red_point = calcIntersectionWithBBox(center, nearest_red_bbox);
        drawArrow(center, red_point, cv::Scalar(0, 0, 255));
    } else {
        red_point = cv::Point(center.x + 125, center.y);
        drawArrow(center, red_point, cv::Scalar(0, 0, 255));
    }

    // 파란 화살표
    cv::Point blue_point;
    if (blue_found) {
        blue_point = calcIntersectionWithBBox(center, nearest_blue_bbox);
        drawArrow(center, blue_point, cv::Scalar(255, 0, 0));
    } else {
        blue_point = cv::Point(center.x + 125, center.y);
        drawArrow(center, blue_point, cv::Scalar(255, 0, 0));
    }

    // 에러값 계산
    double angle_red = red_found ? calculateAngle(center, red_point) : 0.0;
    double angle_blue = blue_found ? calculateAngle(center, blue_point) : 0.0;

    if (!red_found) angle_red = 0.0;
    if (!blue_found) angle_blue = 0.0;

    double diff = std::fabs(angle_red - angle_blue);
    if (diff > 180) diff = 360 - diff;

    double error_angle = (angle_red + angle_blue) / 2.0;
    if (std::fabs(angle_red - angle_blue) > 180) {
        error_angle += 180;
        if (error_angle >= 360) error_angle -= 360;
    }

    // 녹색 화살표 그리기 (에러값 방향, 길이 125)
    cv::Point error_end = calcPointFromAngleLength(center, error_angle, 125);
    drawArrow(center, error_end, cv::Scalar(0, 255, 0));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Error Angle: %.2f degrees", error_angle);

    cv::imshow("Processed Image with Arrows", img);
    cv::waitKey(1);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("image_listener");

    subscription = node->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/image/compressed", 10, image_callback);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}