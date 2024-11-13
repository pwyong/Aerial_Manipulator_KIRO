// Aerial Manipulation Project
// 24.11.13 winch controller start

#ifndef WINCH_CONTROL
#define WINCH_CONTROL

#define DEG2RAD M_PI / 180.0
#define RAD2DEG 180.0 / M_PI;

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

#include <chrono>
#include <vector>
#include <cmath>
#include <string>
#include <iostream>
#include <sstream>

using std::vector;
using namespace std::chrono_literals;
using std::placeholders::_1;

namespace winch_controller
{
    class WinchControl
    {
    public:
        explicit WinchControl(rclcpp::Node::SharedPtr node);

    private:
        // ros2
        rclcpp::Node::SharedPtr node_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr attitude_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr winch_cmd_publisher_;

        // message
        std_msgs::msg::Float32MultiArray winch_cmd_;

        void winch_cmd_publisher_callback();
    };
} // namespace winch_controller

#endif