// Aerial Manipulation Project
// 24.11.05 Just Drone PID flight controller without ros2 function
// 24.11.11 Using SAM mechanical schemetic data -> No self-flight

#ifndef PROPULSION_CONTROL
#define PROPULSION_CONTROL

#define DEG2RAD M_PI / 180.0
#define RAD2DEG 180.0 / M_PI;

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

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

namespace platform_control
{
    class PropulsionControl
    {
    public:
        explicit PropulsionControl(rclcpp::Node::SharedPtr node);

        // PID flight controller -> Desired Wrench 생성 (추후 다른 제어기 사용)
        void flight_pid_control(Eigen::Vector3d cur_position, Eigen::Vector3d cur_attitude);

        Eigen::Matrix<double, 8, 1> Force_to_PWM(Eigen::Matrix<double, 8, 1> thrust);

        // PID gain (파라미터화 예정)
        double roll_kp_, roll_ki_, roll_kd_;
        double pitch_kp_, pitch_ki_, pitch_kd_;
        double yaw_kp_, yaw_ki_, yaw_kd_;
        double X_kp_, X_ki_, X_kd_;
        double Y_kp_, Y_ki_, Y_kd_;
        double Z_kp_, Z_ki_, Z_kd_;

    private:
        // Desired Wrench로부터 Desired thrust 도출
        void get_allocation_matrix();
        void control_allocation();
        Eigen::Matrix3d x_axis_rotation_matrix(double radian);
        Eigen::Matrix3d y_axis_rotation_matrix(double radian);
        Eigen::Matrix3d z_axis_rotation_matrix(double radian);

        Eigen::Matrix<double, 6, 1> wrench_; // [Force Torque]
        Eigen::Matrix<double, 6, 8> allocation_matrix_;
        Eigen::Matrix<double, 8, 1> thrust_;
        Eigen::Matrix<double, 8, 6> pinv_allocation_matrix_;

        // PID 제어를 위한 변수
        Eigen::Vector3d desired_position_;
        Eigen::Vector3d desired_attitude_;
        Eigen::Vector3d previous_position_error_;
        Eigen::Vector3d previous_attitude_error_;
        Eigen::Vector3d error_position_integ_;
        Eigen::Vector3d error_attitude_integ_;
        rclcpp::Time previous_time_;

        // Aerial Platform parameter
        double r_ = 0.75;
        Eigen::Matrix<double, 8, 1> alpha_;
        double zeta_ = 0.02; // BLDC Thrust force-Torque ratio
        double g = 9.80665;  // gravitational acceleration (m/s^2)
        double mass = 77.0;  //(kg)
        Eigen::Vector3d gravity_force_;

        // Sensor data
        Eigen::Vector3d position_;
        Eigen::Vector3d linear_velocity_;
        Eigen::Vector3d acceleration_;
        Eigen::Vector3d attitude_;
        Eigen::Vector3d angular_velocity_;

        // ros2
        rclcpp::Node::SharedPtr node_;
        rclcpp::TimerBase::SharedPtr timer_;
    };
} // namespace platform_control

#endif