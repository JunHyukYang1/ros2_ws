#include "rclcpp/rclcpp.hpp"                            // ROS2 기본 헤더 포함
#include "sensor_msgs/msg/compressed_image.hpp"         // 압축 이미지 메시지 타입 포함
#include "std_msgs/msg/int32.hpp"                       // 정수 메시지 타입 포함
#include <opencv2/opencv.hpp>                           // OpenCV 헤더 포함
#include <cmath>                                        // 수학 함수 포함

using std::placeholders::_1;
using namespace cv;

// 라인 에러 노드 클래스 정의
class LineErrorNode : public rclcpp::Node
{
public:
  // 생성자
  LineErrorNode() : Node("line_error_node"), main_point_(Point(-1, -1)), gain_(1.0), MINDISTANCE_(50), lost_count_(0), MAX_LOST_FRAMES(10), velocity_(0, 0)
  {
    // 압축 이미지 구독자 생성
    subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
      "/image/compressed", 10, std::bind(&LineErrorNode::image_callback, this, _1));
    
    // 라인 에러 퍼블리셔 생성
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("/line_error", 10);

    // 디버깅용 이미지 윈도우 생성
    cv::namedWindow("Raw Image", cv::WINDOW_NORMAL);
    cv::namedWindow("Binary ROI", cv::WINDOW_NORMAL);
  }

private:
  // ROI 자르고 이진화 처리 함수
  void setFrame(Mat& frame)
  {
    frame = frame(Rect(Point(0, frame.rows * 3 / 4), Point(frame.cols, frame.rows))); // 하단 1/4 ROI
    cvtColor(frame, frame, COLOR_BGR2GRAY);                                         // 그레이스케일 변환
    frame += Scalar(100) - mean(frame);                                             // 밝기 보정
    threshold(frame, frame, 0, 255, THRESH_BINARY | THRESH_OTSU);                   // OTSU 이진화
  }

  // main_point와 가장 가까운 라벨 인덱스 찾기
  int findMinIndex(const Mat& stats, const Mat& centroids, int labels, Point& po, int MINDISTANCE)
  {
    int index = 0;
    double mindistance = MINDISTANCE;
    
    for (int i = 1; i < labels; ++i)
    {
      if (stats.at<int>(i, CC_STAT_AREA) < 50) continue;
      double dx = po.x - centroids.at<double>(i, 0);
      double dy = po.y - centroids.at<double>(i, 1);
      double distance = sqrt(dx * dx + dy * dy);
      if (distance < mindistance)
      {
        mindistance = distance;
        index = i;
      }
    }

    if (mindistance < MINDISTANCE)
    {
      Point new_point(static_cast<int>(centroids.at<double>(index, 0)),
                      static_cast<int>(centroids.at<double>(index, 1)));
      if (po != new_point)
      {
        po = new_point;
      }
    }
    return index;
  }

  // 바운딩 박스 및 중심점 시각화 함수
  void drawBoundingBox(Mat& frame, const Mat& stats, const Mat& centroids, int labels, int target_index, Point po)
  {
    cvtColor(frame, frame, COLOR_GRAY2BGR);
    for (int i = 1; i < labels; ++i)
    {
      Scalar color;
      if (i == target_index) color = Scalar(0, 0, 255);                     // 타겟: 빨간색
      else if (stats.at<int>(i, CC_STAT_AREA) < 50) color = Scalar(0, 255, 255); // 너무 작음: 노란색
      else color = Scalar(255, 0, 0);                                       // 일반: 파란색

      rectangle(frame,
        Rect(stats.at<int>(i, CC_STAT_LEFT),
             stats.at<int>(i, CC_STAT_TOP),
             stats.at<int>(i, CC_STAT_WIDTH),
             stats.at<int>(i, CC_STAT_HEIGHT)),
        color, 2);

      circle(frame,
        Point(static_cast<int>(centroids.at<double>(i, 0)),
              static_cast<int>(centroids.at<double>(i, 1))),
        3, color, -1);
    }

    if (target_index == 0)
    {
      rectangle(frame, Rect(po.x, po.y, 3, 3), Scalar(0, 0, 255), 2);
    }
  }

  // 이미지 콜백 함수 (핵심 처리 루틴)
  void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
  {
    try
    {
      // 압축 이미지 디코딩
      Mat decoded_image = imdecode(Mat(msg->data), IMREAD_COLOR);
      if (decoded_image.empty())
      {
        RCLCPP_WARN(this->get_logger(), "Decoded image is empty.");
        return;
      }

      imshow("Raw Image", decoded_image); // 원본 이미지 출력

      Mat roi_image = decoded_image.clone();
      setFrame(roi_image); // ROI 자르고 이진화

      Mat labels, stats, centroids;
      int n_labels = connectedComponentsWithStats(roi_image, labels, stats, centroids);

      // main_point 초기화 (처음 한 번)
      if (main_point_ == Point(-1, -1))
        main_point_ = Point(roi_image.cols / 2, roi_image.rows / 2);

      // 가장 가까운 중심점 인덱스 탐색
      int target_label = findMinIndex(stats, centroids, n_labels, main_point_, MINDISTANCE_);

      if (target_label != 0) // 타겟 라인이 존재할 때
      {
        // 이전 위치와 현재 위치 차이로 속도 계산 (이동 벡터)
        Point2f current_pos(static_cast<float>(main_point_.x), static_cast<float>(main_point_.y));
        Point2f new_pos(static_cast<float>(centroids.at<double>(target_label, 0)), static_cast<float>(centroids.at<double>(target_label, 1)));
        velocity_ = new_pos - current_pos;
        main_point_ = Point(static_cast<int>(new_pos.x), static_cast<int>(new_pos.y));
        lost_count_ = 0; // 라인 발견해서 초기화 실패 카운터 리셋
      }
      else // 타겟 라인 없을 때
      {
        if (lost_count_ < MAX_LOST_FRAMES)
        {
          main_point_ += Point(static_cast<int>(velocity_.x), static_cast<int>(velocity_.y)); // 이전 속도로 위치 예측 유지
          lost_count_++;
        }
        else
        {
          // 초기화 하지 않고 위치 유지, 속도 0으로 세팅
          velocity_ = Point2f(0, 0);
          // main_point_는 변경하지 않음
        }
      }

      // 라인 중심과 ROI 중앙과의 좌우 오차 계산
      double error = (roi_image.cols / 2.0 - main_point_.x) * gain_;

      // 에러 메시지 발행
      std_msgs::msg::Int32 err_msg;
      err_msg.data = static_cast<int>(error);
      publisher_->publish(err_msg);

      RCLCPP_INFO(this->get_logger(), "Line error: %d", err_msg.data);

      // 시각화
      drawBoundingBox(roi_image, stats, centroids, n_labels, target_label, main_point_);

      imshow("Binary ROI", roi_image);
      cv::waitKey(1);
    }
    catch (cv::Exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;

  Point main_point_;        // 메인 라인 기준점
  double gain_;             // 에러 비례 상수
  int MINDISTANCE_;         // 기준점 근처 최소 거리 임계값

  int lost_count_;          // 라인 미검출 연속 카운터
  const int MAX_LOST_FRAMES; // 최대 연속 미검출 프레임 수

  Point2f velocity_;        // 기준점 이동 속도(이동 벡터)
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineErrorNode>());
  rclcpp::shutdown();
  return 0;
}