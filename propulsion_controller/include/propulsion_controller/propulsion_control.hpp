//Aerial Manipulation Project
//24.11.05 Just Drone PID flight controller without ros2 function

#ifndef PROPULSION_CONTROL
#define PROPULSION_CONTROL

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
        explicit PropulsionControl();

        //PID flight controller -> Desired Wrench 생성 (추후 다른 제어기 사용)
        void flight_control(vector<double> cur_position, vector<double> cur_attitude);


        //PID gain (파라미터화 예정)
        double roll_kp_, roll_ki_, roll_kd;
        double pitch_kp_, pitch_ki_, pitch_kd_;
        double yaw_kp_, yaw_ki_, yaw_kd_;
        double X_kp_, X_ki_, X_kd_;
        double Y_kp_, Y_ki_, Y_kd_;
        double Z_kp_, Z_ki_, Z_kd_;

    private:
        //Desired Wrench로부터 Desired thrust 도출
        void control_allocation();

        Eigen::VectorXd wrench_;
        Eigen::MatrixXd allocation_matrix_;
        Eigen::VectorXd thrust_;

        //PID 제어를 위한 변수
        double error_roll_, error_pitch_, error_yaw_;
        double error_X_, error_Y_, error_Z_;
        double error_roll_integ_, error_pitch_integ_, error_yaw_integ_;
        double error_X_integ_, error_Y_integ_, error_Z_integ_;
        

    };
} // namespace platform_control

#endif