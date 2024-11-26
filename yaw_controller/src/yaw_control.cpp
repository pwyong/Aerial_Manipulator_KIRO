#include "yaw_controller/yaw_control.hpp"

namespace yaw_controller
{
    YawControl::YawControl(rclcpp::Node::SharedPtr node) : node_(node)
    {
        timer_ = node_->create_wall_timer(5ms, std::bind(&YawControl::yaw_PD_control, this));

        desired_yaw_torque_publisher_ = node_->create_publisher<std_msgs::msg::Float32>("desired_yaw_torque", 10);

        attitude_subscription_ = node->create_subscription<geometry_msgs::msg::Vector3>("attitude", 10, std::bind(&YawControl::attitude_callback, this, _1));
        desired_attitude_subscription_ = node->create_subscription<geometry_msgs::msg::Vector3>("desired_attitude", 10, std::bind(&YawControl::desired_attitude_callback, this, _1));
        angular_velocity_subscription_ = node->create_subscription<geometry_msgs::msg::Vector3>("angular_velocity", 10, std::bind(&YawControl::angular_velocity_callback, this, _1));
    }

    void YawControl::yaw_PD_control()
    {
        double yaw_error = desired_yaw_ - current_yaw_;

        desired_yaw_torque_.data = Kp_ * yaw_error - Kd_ * yaw_rate_;
    }

    void YawControl::timer_callback()
    {
        yaw_PD_control();

        desired_yaw_torque_publisher_->publish(desired_yaw_torque_);
    }

    void YawControl::attitude_callback(const geometry_msgs::msg::Vector3 &msg)
    {
        current_yaw_ = msg.z;
    }

    void YawControl::desired_attitude_callback(const geometry_msgs::msg::Vector3 &msg)
    {
        desired_yaw_ = msg.z;
    }

    void YawControl::angular_velocity_callback(const geometry_msgs::msg::Vector3 &msg)
    {
        yaw_rate_ = msg.z;
    }

} // namespace yaw_controller