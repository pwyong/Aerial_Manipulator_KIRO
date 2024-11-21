// Aerial Manipulation Project
// 24.11.05 Just Drone PID flight controller without ros2 function
// 24.11.11 Using SAM mechanical schemetic data -> No self-flight

#ifndef PROPULSION_CONTROL
#define PROPULSION_CONTROL

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

namespace propulsion_controller
{
    class PropulsionControl
    {
    public:
        explicit PropulsionControl(rclcpp::Node::SharedPtr node);

    private:
        // Thrust와 PWM 신호 사이의 관계식
        Eigen::Matrix<double, 8, 1> Force_to_PWM(Eigen::Matrix<double, 8, 1> thrust);

        // 플랫폼의 추진 시스템의 배치에 대한 Control Allocation 행렬 도출
        void get_allocation_matrix();

        // Get Nullspace Vector (모든 추력에 대한 음수 값 양수 보정)
        Eigen::VectorXd compute_nullspace_vector();
        Eigen::Matrix<double, 8, 1> nullspace_vector_;

        // Desired Wrench로부터 Desired thrust 계산
        double thrust_low_limit = 0.0;
        void control_allocation();

        Eigen::Matrix3d x_axis_rotation_matrix(double radian);
        Eigen::Matrix3d y_axis_rotation_matrix(double radian);
        Eigen::Matrix3d z_axis_rotation_matrix(double radian);

        Eigen::Matrix<double, 6, 1> wrench_; // [Force Torque]
        Eigen::Matrix<double, 6, 8> allocation_matrix_;
        Eigen::Matrix<double, 8, 1> thrust_;
        Eigen::Matrix<double, 8, 6> pinv_allocation_matrix_;

        // Aerial Platform parameter
        double r_ = 0.75;
        Eigen::Matrix<double, 8, 1> alpha_;
        double zeta_ = 0.02; // BLDC Thrust Force-Torque ratio

        // Sensor data
        Eigen::Vector3d position_;
        Eigen::Vector3d linear_velocity_;
        Eigen::Vector3d acceleration_;
        Eigen::Vector3d attitude_;
        Eigen::Vector3d angular_velocity_;

        // ros2
        rclcpp::Node::SharedPtr node_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr thrust_publisher_; // thrust data or pwm data
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wrench_publisher_; // test
        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr desired_wrench_subscription_;

        // topic callback
        void thrust_publisher_callback();
        void desired_wrench_callback(const std_msgs::msg::Float32MultiArray &msg);

        // message
        std_msgs::msg::Float32MultiArray thrust_msg;
        std_msgs::msg::Float32MultiArray wrench_msg; //test
    };
} // namespace platform_control

#endif