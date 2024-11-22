#include "oscillation_damping_controller/oscillation_damping_control.hpp"

namespace oscillation_damping_controller
{
    DampingControl::DampingControl(rclcpp::Node::SharedPtr node) : node_(node)
    {

        desired_wrench_.data.resize(6);
        desired_wrench_publisher_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("desired_wrench", 10);

        attitude_subscription_ = node_->create_subscription<geometry_msgs::msg::Vector3>("attitude", 10, std::bind(&DampingControl::attitude_callback, this, _1));
        angular_velocity_subscription_ = node_->create_subscription<geometry_msgs::msg::Vector3>("angular_velocity", 10, std::bind(&DampingControl::angular_velocity_callback, this, _1));

        timer_ = node_->create_wall_timer(5ms, std::bind(&DampingControl::timer_callback, this));
    }

    void DampingControl::damping_control()
    {
        
    }

    void DampingControl::timer_callback()
    {
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