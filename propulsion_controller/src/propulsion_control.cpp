#include "propulsion_controller/propulsion_control.hpp"

namespace platform_control
{
    PropulsionControl::PropulsionControl(rclcpp::Node::SharedPtr node) : node_(node)
    {

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

    void PropulsionControl::flight_control(Eigen::Vector3d cur_position, Eigen::Vector3d cur_attitude)
    {
        rclcpp::Time current_time = node_->get_clock()->now();
        double delta_time = (current_time - previous_time_).seconds();

        Eigen::Vector3d error_position = desired_position_ - cur_position;
        Eigen::Vector3d error_attitude = desired_attitude_ - cur_attitude;

        previous_position_error_ = error_position;
        previous_attitude_error_ = error_attitude;

        previous_time_ = current_time;
    }
}