#include "oscillation_damping_controller/oscillation_damping_control.hpp"

namespace oscillation_damping_controller
{
    DampingControl::DampingControl(rclcpp::Node::SharedPtr node) : node_(node)
    {
        lpf_cutoff_frequency_computation();

        desired_wrench_.data.resize(6);
        for (int i = 0; i < 6; i++)
        {
            desired_wrench_.data[i] = 0.0;
        }

        desired_wrench_publisher_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("desired_wrench", 10);

        attitude_subscription_ = node_->create_subscription<geometry_msgs::msg::Vector3>("attitude", 10, std::bind(&DampingControl::attitude_callback, this, _1));
        angular_velocity_subscription_ = node_->create_subscription<geometry_msgs::msg::Vector3>("angular_velocity", 10, std::bind(&DampingControl::angular_velocity_callback, this, _1));

        timer_ = node_->create_wall_timer(5ms, std::bind(&DampingControl::timer_callback, this));

        previous_time_ = node_->get_clock()->now();
    }

    void DampingControl::oscillation_damping_control()
    {
        // Roll (x axis oscillation damping)
        double f_by_des = -Kv_roll * (l1 * roll_rate_lpf + l2 * angular_velocity_.x);
        double tau_r_des = -Kw_roll * angular_velocity_.x;

        // Pitch (y axis oscillation damping)
        double f_bx_des = -Kv_pitch * (l1 * pitch_rate_lpf + l2 * angular_velocity_.y);
        double tau_p_des = -Kw_pitch * angular_velocity_.y;

        desired_wrench_.data[0] = f_bx_des;
        desired_wrench_.data[1] = f_by_des;
        desired_wrench_.data[2] = 0;
        desired_wrench_.data[3] = tau_r_des;
        desired_wrench_.data[4] = tau_p_des;
        desired_wrench_.data[5] = 0; // -> Yaw controller result
    }

    void DampingControl::lpf_cutoff_frequency_computation()
    {
        double fast_frequency = sqrt(g * (m1 + m2) / (8.0 * pow(M_PI, 2.0) * m1 * l1 * l2) * (l1 + l2 + sqrt(pow(l1 + l2, 2) - 4 * m1 * l1 * l2 / (m1 + m2))));
        double slow_frequency = sqrt(g * (m1 + m2) / (8.0 * pow(M_PI, 2.0) * m1 * l1 * l2) * (l1 + l2 - sqrt(pow(l1 + l2, 2) - 4 * m1 * l1 * l2 / (m1 + m2))));

        cutoff_frequency_ = (fast_frequency + slow_frequency) / 2.0;
    }

    void DampingControl::angular_velocity_lpf()
    {
        rclcpp::Time current_time = node_->get_clock()->now();
        double delta_time = (current_time - previous_time_).seconds();
        previous_time_ = current_time;

        double x_wb_roll_dot, x_wb_pitch_dot;

        x_wb_roll_dot = -cutoff_frequency_ * x_wb_roll + angular_velocity_.x;
        x_wb_roll += delta_time * x_wb_roll_dot;
        roll_rate_lpf = cutoff_frequency_ * x_wb_roll;

        x_wb_pitch_dot = -cutoff_frequency_ * x_wb_pitch + angular_velocity_.y;
        x_wb_pitch += delta_time * x_wb_pitch_dot;
        pitch_rate_lpf = cutoff_frequency_ * x_wb_pitch;
    }

    void DampingControl::timer_callback()
    {
        angular_velocity_lpf();

        oscillation_damping_control();

        desired_wrench_publisher_->publish(desired_wrench_);
    }

    void DampingControl::attitude_callback(const geometry_msgs::msg::Vector3 &msg)
    {
        attitude_ = msg;
    }

    void DampingControl::angular_velocity_callback(const geometry_msgs::msg::Vector3 &msg)
    {
        angular_velocity_ = msg;
    }

}