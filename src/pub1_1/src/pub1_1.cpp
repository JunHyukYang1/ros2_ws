#include "rclcpp/rclcpp.hpp"            // ROS2의 모든 헤더파일 포함
#include "std_msgs/msg/int32.hpp"       // Int32 인터페이스를 사용하기 위한 별도 헤더파일 포함
#include <memory>                       // 스마트포인터를 사용하기 위한 헤더파일
#include <chrono>                       // 시간 관련 기능을 사용하기 위한 헤더파일

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);         // ROS2 초기화
	
    auto node = std::make_shared<rclcpp::Node>("node_pub1_1");       // 노드명 node_pub1_!을 가지는 노드객체 동적생성
	auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));  // History옵션을 KEEP_LAST, depth=10으로 설정하고 나머지는 디폴트설정
	auto mypub = node->create_publisher<std_msgs::msg::Int32>("topic_pub1_1", qos_profile );                         // "topic_pub1_1" 토픽에 Int32 타입 발행자 생성
	
    std_msgs::msg::Int32 message;          // Int32 타입 메시지 객체 생성
	message.data = 0;                      // 메세지 데이터 정수값 0으로 초기화
	
    rclcpp::WallRate loop_rate(1.0);    // 반복주파수를 저장하는 객체(단위 Hz), 1Hz 주기 제어
	
    while(rclcpp::ok())              // ROS2 시스템 정상 작동 동안 반복
	{
		message.data++;        // 반복할때마다 메세지 데이터 값이 1씩 증가함
		RCLCPP_INFO(node->get_logger(), "Publish: %d", message.data);   // 로그 출력 (INFO 레벨)
		mypub->publish(message);       // 메시지 발행
		//rclcpp::spin_some(node);
		loop_rate.sleep(); //반복주파수에서 남은 시간 만큼 sleep
	}
	
    rclcpp::shutdown();        // ROS2 시스템 종료
	return 0;                  // 프로그램 정상 종료
}