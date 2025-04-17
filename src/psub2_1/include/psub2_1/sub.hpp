#ifndef _SUB_HPP_
#define _SUB_HPP_
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <functional>
#include <memory>
using std::placeholders::_1;

class Sub : public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
    void subscribe_msg(const std_msgs::msg::Int32::SharedPtr msg) const;
public:
    Sub();
};

#endif //_SUB_HPP_