#include "winch_controller/winch_control.hpp"

namespace winch_controller
{
    WinchControl::WinchControl(rclcpp::Node::SharedPtr node) : node_(node)
    {
        adm_state_ = Eigen::MatrixXd::Zero(2, 3);
        adm_input_ = Eigen::Vector3d::Zero(3);

        desired_cable_length_ = Eigen::Vector3d::Ones(3);
        M_adm_ << 1, 2, 3; // test
        D_adm_ << 5, 4, 3; // test
        for (int i = 0; i < winch_cable_position.cols(); i++)
        {
            winch_cable_position.col(i) << winch_displacement_radius / 2.0 * cos((gamma + 120.0 * (i + 1)) * DEG2RAD), winch_displacement_radius / 2.0 * sin((gamma + 120.0 * (i + 1)) * DEG2RAD), 1.0;
        }

        winch_cmd_.data.resize(3);

        winch_cmd_publisher_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("winch_cmd", 10);
        timer_ = node_->create_wall_timer(5ms, std::bind(&WinchControl::winch_cmd_publisher_callback, this));

        attitude_subscription_ = node->create_subscription<geometry_msgs::msg::Vector3>("current_attitude", 10, std::bind(&WinchControl::attitude_callback, this, _1));
        desired_attitude_subscription_ = node->create_subscription<geometry_msgs::msg::Vector3>("desired_attitude", 10, std::bind(&WinchControl::desired_attitude_callback, this, _1));
        quasi_joint_torque_subscription_ = node->create_subscription<geometry_msgs::msg::Vector3>("quasi_joint_torque", 10, std::bind(&WinchControl::quasi_joint_torque_callback, this, _1));

        previous_time_ = node_->get_clock()->now();
    }

    void WinchControl::roll_pitch_control()
    {
    }

    void WinchControl::CoM_compensation_control()
    {
        admittance_interface();
        inverse_kinematics();
    }

    void WinchControl::admittance_interface()
    {
        rclcpp::Time current_time = node_->get_clock()->now();
        double delta_time = (current_time - previous_time_).seconds();

        for (int i = 0; i < 3; i++)
        {
            Eigen::Vector2d state_dot = Eigen::VectorXd::Zero(2);
            Eigen::Matrix2d A;
            Eigen::Matrix<double, 2, 1> B;
            Eigen::Matrix<double, 1, 2> C;
            A << 0, 1,
                0, -D_adm_(i) / M_adm_(i);
            B << 0, 1;
            C << 1 / M_adm_(i), 0;
            state_dot =  A * adm_state_.col(i) + B * adm_input_(i);
            //RCLCPP_INFO(node_->get_logger(), "%lf", adm_input_(i));
            adm_state_.col(i) += delta_time * state_dot;
            desired_platform_center_position_(i) = C * adm_state_.col(i);
        }

        previous_time_ = node_->get_clock()->now();
    }

    void WinchControl::inverse_kinematics()
    {
        for (int i = 0; i < winch_cable_position.cols(); i++)
        {
            Eigen::Vector3d cable_vector = desired_platform_center_position_ + z_axis_rotation_matrix(attitude_.z) * y_axis_rotation_matrix(attitude_.y) * x_axis_rotation_matrix(attitude_.x) * winch_cable_position.col(i); // eq (5.19)
            desired_cable_length_(i) = cable_vector.norm();
            if (desired_cable_length_(i) > cable_max_limit)
                desired_cable_length_(i) = cable_max_limit;
            if (desired_cable_length_(i) < cable_min_limit)
                desired_cable_length_(i) = cable_min_limit;
            winch_cmd_.data[i] = desired_cable_length_(i);
        }
    }

    void WinchControl::winch_cmd_publisher_callback()
    {
        if (position_hold)
        {
            if (manipulation_mode)
            {
                CoM_compensation_control();
            }
            else
            {
                roll_pitch_control();
            }
        }
        else
        {
            for (int i = 0; i < 3; i++)
            {
                winch_cmd_.data[i] = 1.0;
            }
        }

        winch_cmd_publisher_->publish(winch_cmd_);
    }

    void WinchControl::attitude_callback(const geometry_msgs::msg::Vector3 &msg)
    {
        attitude_ = msg;
    }

    void WinchControl::desired_attitude_callback(const geometry_msgs::msg::Vector3 &msg)
    {
        desired_attitude_ = msg;
    }

    void WinchControl::quasi_joint_torque_callback(const geometry_msgs::msg::Vector3 &msg)
    {
        quasi_joint_torque_ = msg;
        adm_input_ << msg.x, msg.y, msg.z;
    }

    Eigen::Matrix3d WinchControl::x_axis_rotation_matrix(double radian)
    {
        Eigen::Matrix3d Rotx;

        Rotx << 1, 0, 0,
            0, cos(radian), -sin(radian),
            0, sin(radian), cos(radian);

        return Rotx;
    }
    Eigen::Matrix3d WinchControl::y_axis_rotation_matrix(double radian)
    {
        Eigen::Matrix3d Roty;

        Roty << cos(radian), 0, sin(radian),
            0, 1, 0,
            -sin(radian), 0, cos(radian);

        return Roty;
    }
    Eigen::Matrix3d WinchControl::z_axis_rotation_matrix(double radian)
    {
        Eigen::Matrix3d Rotz;

        Rotz << cos(radian), -sin(radian), 0,
            sin(radian), cos(radian), 0,
            0, 0, 1;

        return Rotz;
    }

}