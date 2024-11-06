// Aerial Manipulation Project
// 24.11.05 Just Drone PID flight controller without ros2 function

#ifndef PROPULSION_CONTROL
#define PROPULSION_CONTROL

#define DEG2RAD M_PI / 180.0
#define RAD2DEG 180.0 / M_PI;

#include "rclcpp/rclcpp.hpp"

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

#include <chrono>
#include <vector>
#include <cmath>
#include <string>

using std::vector;

namespace platform_control
{
    class PropulsionControl
    {
    public:
        explicit PropulsionControl(rclcpp::Node::SharedPtr node);

        // PID flight controller -> Desired Wrench 생성 (추후 다른 제어기 사용)
        void flight_control(Eigen::Vector3d cur_position, Eigen::Vector3d cur_attitude);

        // PID gain (파라미터화 예정)
        double roll_kp_, roll_ki_, roll_kd;
        double pitch_kp_, pitch_ki_, pitch_kd_;
        double yaw_kp_, yaw_ki_, yaw_kd_;
        double X_kp_, X_ki_, X_kd_;
        double Y_kp_, Y_ki_, Y_kd_;
        double Z_kp_, Z_ki_, Z_kd_;

    private:
        // Desired Wrench로부터 Desired thrust 도출
        void control_allocation();

        Eigen::Matrix<double, 6, 1> wrench_; // [Force^T Torque^T]^T
        Eigen::Matrix<double, 6, 8> allocation_matrix_;
        Eigen::Matrix<double, 8, 1> thrust_;
        Eigen::Matrix<double, 8, 6> pinv_allocation_matrix_;
        

        // PID 제어를 위한 변수
        double error_roll_integ_, error_pitch_integ_, error_yaw_integ_;
        double error_X_integ_, error_Y_integ_, error_Z_integ_;
        Eigen::Vector3d desired_position_;
        Eigen::Vector3d desired_attitude_;
        Eigen::Vector3d previous_position_error_;
        Eigen::Vector3d previous_attitude_error_;


        // Aerial Platform parameter
        double r_ = 2.0;
        double alpha_ = 30 * DEG2RAD;
        double sa_ = sin(alpha_);
        double ca_ = cos(alpha_);
        double zeta_ = 0.01; // BLDC Thrust force-Torque ratio

        // ros2
        rclcpp::Node::SharedPtr node_;
        rclcpp::Time previous_time_;
    };
} // namespace platform_control

#endif