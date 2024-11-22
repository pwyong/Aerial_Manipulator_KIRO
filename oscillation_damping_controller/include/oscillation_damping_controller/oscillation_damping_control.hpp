//24.11.22 Oscillation damping controller w. LQR


#ifndef OSCILLATION_DAMPING_CONTROL
#define OSCILLATION_DAMPING_CONTROL

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

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace oscillation_damping_controller
{
    class DampingControl
    {
    public:
        explicit DampingControl(rclcpp::Node::SharedPtr node);

    private:
        void damping_control();

        // parameter
        double Kv = 1.0;
        double kw = 10.0;

        // ros2
        rclcpp::Node::SharedPtr node_;
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr desired_wrench_publisher_;
        
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr attitude_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angular_velocity_subscription_;

        // callback
        void timer_callback();
        void attitude_callback(const geometry_msgs::msg::Vector3 &msg);
        void angular_velocity_callback(const geometry_msgs::msg::Vector3 &msg);

        // msg
        std_msgs::msg::Float32MultiArray desired_wrench_;
        geometry_msgs::msg::Vector3 attitude_;
        geometry_msgs::msg::Vector3 angular_velocity_;

    };
} // namespace oscillation_damping_controller

#endif