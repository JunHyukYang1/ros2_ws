#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <memory>
#include <chrono>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("node_pub1_2");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    auto mypub = node->create_publisher<geometry_msgs::msg::Vector3>("topic_pub1_2", qos_profile );
    
    geometry_msgs::msg::Vector3 message;
    
    rclcpp::WallRate loop_rate(1.0); //반복주파수를 저장하는 객체(단위 Hz)
    
    while(rclcpp::ok())
    {
        std::cout << "Enter x, y, z values (separated by space): ";
        std::cin >> message.x >> message.y >> message.z;

        // 퍼블리시 및 로그 출력
        RCLCPP_INFO(node->get_logger(), "Publishing: x=%.2f, y=%.2f, z=%.2f", message.x, message.y, message.z);
        mypub->publish(message);
    }
    
    rclcpp::shutdown();
    return 0;
}