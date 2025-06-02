#include <memory>
#include <iostream>
#include <algorithm>  // std::clamp 사용을 위한 헤더 추가
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include "dxl_controller/dxl.hpp"  // 실제 Dxl 클래스 헤더 경로로 수정 필요

class DxlController : public rclcpp::Node
{
public:
    DxlController()
    : Node("dxl_controller_node"), dxl()
    {
        // 다이나믹셀 연결 시도
        if (!dxl.open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open Dynamixel port");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Dynamixel port opened");

        // 에러값 구독
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "/line_error", 10,
            std::bind(&DxlController::errorCallback, this, std::placeholders::_1));

        // 초기 속도 0으로 설정
        dxl.setVelocity(0, 0);
    }

    ~DxlController()
    {
        dxl.close();
        RCLCPP_INFO(this->get_logger(), "Dynamixel port closed");
    }

private:
    void errorCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        int error = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received error: %d", error);

        // 기본 전진 속도 및 제어 계수
        const int base_speed = 100;
        const double Kp = 0.5;
        const int MAX_SPEED = 200;

        // 비례 제어 계산
        int left_speed = base_speed - static_cast<int>(Kp * error);
        int right_speed = base_speed + static_cast<int>(Kp * error);

        // ✅ 속도 제한 적용
        left_speed = std::clamp(left_speed, -MAX_SPEED, MAX_SPEED);
        right_speed = std::clamp(right_speed, -MAX_SPEED, MAX_SPEED);

        // 다이나믹셀 속도 설정
        bool result = dxl.setVelocity(left_speed, right_speed);
        if (!result)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to set Dynamixel velocity");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Set velocity - Left: %d, Right: %d", left_speed, right_speed);
        }
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    Dxl dxl;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DxlController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}