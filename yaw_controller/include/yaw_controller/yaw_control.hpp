// Aerial Manipulation Project
// 24.11.26 PD controller for Yaw control

#ifndef YAW_CONTROL
#define YAW_CONTROL

#define DEG2RAD M_PI / 180.0
#define RAD2DEG 180.0 / M_PI;

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
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

namespace yaw_controller
{
    class YawControl
    {
    public:
        explicit YawControl(rclcpp::Node::SharedPtr node);

    private:
        // PD controller
        void yaw_PD_control();

        double desired_yaw_ = 0.0;
        double current_yaw_ = 0.0;
        double yaw_rate_ = 0.0;
        double Kp_ = 1.0;
        double Kd_ = 0.1;


        // ros2
        rclcpp::Node::SharedPtr node_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr desired_yaw_torque_publisher_;

        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr attitude_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr desired_attitude_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr angular_velocity_subscription_;

        //message
        std_msgs::msg::Float32 desired_yaw_torque_;

        // callback
        void timer_callback();
        void attitude_callback(const geometry_msgs::msg::Vector3 &msg);
        void desired_attitude_callback(const geometry_msgs::msg::Vector3 &msg);
        void angular_velocity_callback(const geometry_msgs::msg::Vector3 &msg);
    };
} // namespace yaw_controller

#endif