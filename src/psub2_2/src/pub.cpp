#include "psub2_2/pub.hpp"

Pub::Pub() : Node("mypub"), count_(0)
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("mytopic", qos_profile);
    timer_ = this->create_wall_timer(1s, std::bind(&Pub::publish_msg, this));
}

void Pub::publish_msg()
{
    static double x = 0, y = 0, z = 0;
    auto msg = geometry_msgs::msg::Vector3();
    msg.x = x++;
    msg.y = y++;
    msg.z = z++;
    RCLCPP_INFO(this->get_logger(), "Published message: 'x: %f, y: %f, z: %f'", msg.x, msg.y, msg.z);
    pub_->publish(msg);
}