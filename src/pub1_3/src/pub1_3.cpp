#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <termios.h>
#include <unistd.h>

char getKey() {
    struct termios oldt, newt;
    char ch;
    
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    
    ch = getchar();
    
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	
	auto node = std::make_shared<rclcpp::Node>("node_pub1_3");
	auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
	auto mypub = node->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel",
	qos_profile );
	
	RCLCPP_INFO(node->get_logger(), "이동 명령: f(전진), b(후진), l(좌회전), r(우회전)");

	while(rclcpp::ok())
	{
		char key = getKey();
        auto twist = geometry_msgs::msg::Twist();

        switch(key) {
            case 'f': twist.linear.x = 2.0; break;
            case 'b': twist.linear.x = -2.0; break;
            case 'l': twist.angular.z = 1.8; break;
            case 'r': twist.angular.z = -1.8; break;
            case 3:   // Ctrl+C 처리
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                mypub->publish(twist);
                rclcpp::shutdown();
                return 0;
            default: continue;
        }

        RCLCPP_INFO(node->get_logger(), 
            "발행 명령: linear.x=%.1f, angular.z=%.1f", 
            twist.linear.x, 
            twist.angular.z
        );
        mypub->publish(twist);
	}

	rclcpp::shutdown();
	return 0;
}