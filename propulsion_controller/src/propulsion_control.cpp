#include "propulsion_controller/propulsion_control.hpp"

namespace platform_control
{
    PropulsionControl::PropulsionControl(rclcpp::Node::SharedPtr node) : node_(node)
    {

        alpha_ << 53.1, -54.2, -126.9, 125.9, 53.1, -54.1, -126.9, 125.9;
        alpha_ = alpha_ * DEG2RAD;

        get_allocation_matrix();
        pinv_allocation_matrix_ = allocation_matrix_.completeOrthogonalDecomposition().pseudoInverse();

        previous_time_ = node_->get_clock()->now();

        thrust_msg.data.resize(8);

        thrust_publisher_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("thrust", 10);
        desired_force_subscription_ = node_->create_subscription<geometry_msgs::msg::Vector3>("desired_force", 10, std::bind(&PropulsionControl::desired_force_callback, this, _1));
        desired_torque_subscription_ = node_->create_subscription<geometry_msgs::msg::Vector3>("desired_torque", 10, std::bind(&PropulsionControl::desired_torque_callback, this, _1));
        timer_ = node_->create_wall_timer(10ms, std::bind(&PropulsionControl::timer_callback, this));
    }

    void PropulsionControl::get_allocation_matrix()
    {
        for (int i = 0; i < 8; i++)
        {
            Eigen::Vector3d motor_pos;
            Eigen::Vector3d thrust_dir;
            double force_torque_ratio = pow(-1, 1) * zeta_;

            motor_pos << r_, 0, 0;
            thrust_dir << 0, 0, 1;

            motor_pos = z_axis_rotation_matrix((i + 1) / 4.0 * M_PI) * motor_pos;
            thrust_dir = z_axis_rotation_matrix((i + 1) / 4.0 * M_PI) * x_axis_rotation_matrix(alpha_(i)) * thrust_dir;

            allocation_matrix_.block<3, 1>(0, i) = thrust_dir;
            allocation_matrix_.block<3, 1>(3, i) = motor_pos.cross(thrust_dir) + force_torque_ratio * thrust_dir;
        }
    }

    void PropulsionControl::control_allocation()
    {
        thrust_ = pinv_allocation_matrix_ * wrench_;
        for (int i = 0; i < 8; i++)
        {
            thrust_msg.data[i] = thrust_(0);
        }
    }

    void PropulsionControl::flight_pid_control(Eigen::Vector3d cur_position, Eigen::Vector3d cur_attitude)
    {
        rclcpp::Time current_time = node_->get_clock()->now();
        double delta_time = (current_time - previous_time_).seconds();

        Eigen::Vector3d error_position = desired_position_ - cur_position;
        Eigen::Vector3d error_attitude = desired_attitude_ - cur_attitude;

        previous_position_error_ = error_position;
        previous_attitude_error_ = error_attitude;

        error_position_integ_ += error_position * delta_time;
        error_attitude_integ_ += error_attitude * delta_time;

        double F_Xd = X_kp_ * error_position(0) + X_ki_ * error_position_integ_(0) - X_kd_ * linear_velocity_(0);
        double F_Yd = Y_kp_ * error_position(1) + Y_ki_ * error_position_integ_(1) - Y_kd_ * linear_velocity_(1);
        double F_Zd = Z_kp_ * error_position(2) + Z_ki_ * error_position_integ_(2) - Z_kd_ * linear_velocity_(2);

        double tau_rd = roll_kp_ * error_attitude(0) + roll_ki_ * error_attitude_integ_(0) - roll_kd_ * angular_velocity_(0);
        double tau_pd = pitch_kp_ * error_attitude(1) + pitch_ki_ * error_attitude_integ_(1) - pitch_kd_ * angular_velocity_(1);
        double tau_yd = yaw_kp_ * error_attitude(2) + yaw_ki_ * error_attitude_integ_(2) - yaw_kd_ * angular_velocity_(2);

        Eigen::Vector3d desired_force_wrt_world;
        desired_force_wrt_world << F_Xd, F_Yd, F_Zd;

        Eigen::Vector3d desired_force_wrt_body = x_axis_rotation_matrix(-cur_attitude(0)) * y_axis_rotation_matrix(-cur_attitude(1)) * z_axis_rotation_matrix(-cur_attitude(2)) * (desired_force_wrt_world - gravity_force_);

        wrench_ << desired_force_wrt_body(0), desired_force_wrt_body(1), desired_force_wrt_body(2), tau_rd, tau_pd, tau_yd;

        control_allocation();

        previous_time_ = current_time;
    }

    Eigen::Matrix3d PropulsionControl::x_axis_rotation_matrix(double radian)
    {
        Eigen::Matrix3d Rotx;

        Rotx << 1, 0, 0,
            0, cos(radian), -sin(radian),
            0, sin(radian), cos(radian);

        return Rotx;
    }
    Eigen::Matrix3d PropulsionControl::y_axis_rotation_matrix(double radian)
    {
        Eigen::Matrix3d Roty;

        Roty << cos(radian), 0, sin(radian),
            0, 1, 0,
            -sin(radian), 0, cos(radian);

        return Roty;
    }
    Eigen::Matrix3d PropulsionControl::z_axis_rotation_matrix(double radian)
    {
        Eigen::Matrix3d Rotz;

        Rotz << cos(radian), -sin(radian), 0,
            sin(radian), cos(radian), 0,
            0, 0, 1;

        return Rotz;
    }

    Eigen::Matrix<double, 8, 1> PropulsionControl::Force_to_PWM(Eigen::Matrix<double, 8, 1> thrust)
    {
        Eigen::Matrix<double, 8, 1> pwm;
        pwm = thrust;
        return pwm;
    }

    void PropulsionControl::timer_callback()
    {
        thrust_publisher_->publish(thrust_msg);
    }

    void PropulsionControl::desired_force_callback(const geometry_msgs::msg::Vector3 &msg)
    {
        wrench_(0) = msg.x; // Fx
        wrench_(1) = msg.y; // Fy
        wrench_(2) = msg.z; // Fz
    }

    void PropulsionControl::desired_torque_callback(const geometry_msgs::msg::Vector3 &msg)
    {
        wrench_(3) = msg.x; // Tx
        wrench_(4) = msg.y; // Ty
        wrench_(5) = msg.z; // Tz
    }
}