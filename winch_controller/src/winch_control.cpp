#include "winch_controller/winch_control.hpp"

namespace winch_controller
{
    WinchControl::WinchControl(rclcpp::Node::SharedPtr node) : node_(node)
    {

        winch_cmd_.data.resize(3);

        winch_cmd_publisher_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("winch_cmd", 10);
        timer_ = node_->create_wall_timer(5ms, std::bind(&WinchControl::winch_cmd_publisher_callback, this));
    }

    void WinchControl::winch_cmd_publisher_callback()
    {
        winch_cmd_publisher_->publish(winch_cmd_);
    }

}