// 24.11.22 Oscillation damping controller
// 24.11.26 Apply 2d planar double pendulum oscillation damping control for roll, pitch (SAM version)

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
        void oscillation_damping_control();

        // parameter
        double Kv_roll = 1.0;
        double Kw_roll = 10.0;
        double Kv_pitch = 1.0;
        double Kw_pitch = 10.0;
        double l1 = 6;
        double l2 = 2.2;
        double m1 = 18.5;
        double m2 = 77;
        double g = 9.81;

        // Low-Pass filter
        void lpf_cutoff_frequency_computation();
        void angular_velocity_lpf();
        double cutoff_frequency_;
        double x_wb_roll = 0.0;
        double x_wb_pitch = 0.0;
        double roll_rate_lpf;
        double pitch_rate_lpf;

        // ros2
        rclcpp::Node::SharedPtr node_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Time previous_time_;

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