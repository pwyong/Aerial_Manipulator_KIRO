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
        // Winch Operation --------------------------------------------
        // roll pitch setting for side plane task
        void roll_pitch_control(); // 측면 작업을 위한 roll, pitch control

        // robot arm operation compensation
        void CoM_compensation_control(); // 로봇 팔 구동 시 무게 중심 이동 보상 (Admittance control + inverse Kinematics)
        void admittance_interface();
        void inverse_kinematics();
        //-------------------------------------------------------

        // Parameter
        Eigen::Matrix3d winch_cable_position;           //[a1, a2, a3]
        double cable_max_limit = 2.0;                   // m
        double cable_min_limit = 0.5;                   // m
        double winch_displacement_radius = 0.831 / 2.0; // m
        double gamma = 52.49;

        // Control Variable
        Eigen::Vector3d desired_platform_center_position_; // x_sp_des
        Eigen::Vector3d desired_cable_length_;             // l
        Eigen::Vector3d desired_CoM_z_wrt_world;           // r
        Eigen::Vector3d desired_CoM_xy_wrt_body;           // d

        // Admittance interface
        rclcpp::Time previous_time_;
        Eigen::Vector3d M_adm_;                 // Mx, My, Mz
        Eigen::Vector3d D_adm_;                 // Dx, Dy, Dz
        Eigen::Matrix<double, 2, 3> adm_state_; // state in X,Y,Z channel
        Eigen::Vector3d adm_input_;

        // Rotational Matrix
        Eigen::Matrix3d x_axis_rotation_matrix(double radian);
        Eigen::Matrix3d y_axis_rotation_matrix(double radian);
        Eigen::Matrix3d z_axis_rotation_matrix(double radian);

        // Mode
        bool manipulation_mode = true; // ground station에서 보낼 값 -> setting 되면 service client로 구조 변경

        // robot state
        bool position_hold = true;

        // ros2
        rclcpp::Node::SharedPtr node_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr winch_cmd_publisher_;

        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr attitude_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr desired_attitude_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr quasi_joint_torque_subscription_; // whole body controller result

        // message
        std_msgs::msg::Float32MultiArray winch_cmd_;
        geometry_msgs::msg::Vector3 attitude_;
        geometry_msgs::msg::Vector3 desired_attitude_;
        geometry_msgs::msg::Vector3 quasi_joint_torque_;

        // callback function
        void winch_cmd_publisher_callback();
        void attitude_callback(const geometry_msgs::msg::Vector3 &msg);
        void desired_attitude_callback(const geometry_msgs::msg::Vector3 &msg);
        void quasi_joint_torque_callback(const geometry_msgs::msg::Vector3 &msg);
    };
} // namespace winch_controller

#endif