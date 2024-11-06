#include "propulsion_controller/propulsion_control.hpp"

namespace platform_control
{
    PropulsionControl::PropulsionControl(rclcpp::Node::SharedPtr node) : node_(node)
    {
        gravity_force_ << 0, 0, mass * g;

        allocation_matrix_ << 0, sa_ / sqrt(2), -sa_, sa_ / sqrt(2), 0, -sa_ / sqrt(2), sa_, -sa_ / sqrt(2),
            sa_, -sa_ / sqrt(2), 0, sa_ / sqrt(2), -sa_, sa_ / sqrt(2), 0, -sa_ / sqrt(2),
            -ca_, -ca_, -ca_, -ca_, -ca_, -ca_, -ca_, -ca_,
            0, -ca_ * r_ / sqrt(2), -ca_ * r_, -ca_ * r_ / sqrt(2), 0, ca_ * r_ / sqrt(2), ca_ * r_, ca_ * r_ / sqrt(2),
            ca_ * r_, ca_ * r_ / sqrt(2), 0, -ca_ * r_ / sqrt(2), -ca_ * r_, -ca_ * r_ / sqrt(2), 0, ca_ * r_ / sqrt(2),
            -zeta_ * ca_ + sa_ * r_, zeta_ * ca_ - sa_ * r_, -zeta_ * ca_ + sa_ * r_, zeta_ * ca_ - sa_ * r_, -zeta_ * ca_ + sa_ * r_, zeta_ * ca_ - sa_ * r_, -zeta_ * ca_ + sa_ * r_, zeta_ * ca_ - sa_ * r_;

        pinv_allocation_matrix_ = allocation_matrix_.completeOrthogonalDecomposition().pseudoInverse();

        previous_time_ = node_->get_clock()->now();
    }

    void PropulsionControl::control_allocation()
    {
        thrust_ = pinv_allocation_matrix_ * wrench_;
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

    Eigen::Matrix3d PropulsionControl::x_axis_rotation_matrix(double radian){
        Eigen::Matrix3d Rotx;

        Rotx << 1,           0,            0,
                0, cos(radian), -sin(radian),
                0, sin(radian),  cos(radian);
        
        return Rotx;
    }
    Eigen::Matrix3d PropulsionControl::y_axis_rotation_matrix(double radian){
        Eigen::Matrix3d Roty;

        Roty << cos(radian),  0, sin(radian),
                          0,  1,           0,
                -sin(radian), 0, cos(radian);
        
        return Roty;
    }
    Eigen::Matrix3d PropulsionControl::z_axis_rotation_matrix(double radian){
        Eigen::Matrix3d Rotz;

        Rotz << cos(radian), -sin(radian), 0,
                sin(radian),  cos(radian), 0,
                          0,            0, 1;
        
        return Rotz;
    }
}