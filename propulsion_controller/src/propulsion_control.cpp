#include "propulsion_controller/propulsion_control.hpp"

namespace propulsion_controller
{
    PropulsionControl::PropulsionControl(rclcpp::Node::SharedPtr node) : node_(node)
    {

        alpha_ << 53.1, -54.2, -126.9, 125.9, 53.1, -54.1, -126.9, 125.9;
        alpha_ = alpha_ * DEG2RAD;

        get_allocation_matrix();
        pinv_allocation_matrix_ = allocation_matrix_.completeOrthogonalDecomposition().pseudoInverse();

        nullspace_vector_ = compute_nullspace_vector();

        thrust_msg.data.resize(8);
        wrench_msg.data.resize(6);

        thrust_publisher_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("thrust", 10);
        wrench_publisher_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("wrench", 10);
        desired_wrench_subscription_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>("desired_wrench", 10, std::bind(&PropulsionControl::desired_wrench_callback, this, _1));
        timer_ = node_->create_wall_timer(5ms, std::bind(&PropulsionControl::thrust_publisher_callback, this));
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

    Eigen::VectorXd PropulsionControl::compute_nullspace_vector()
    {
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(allocation_matrix_, Eigen::ComputeFullV);

        Eigen::MatrixXd V = svd.matrixV();
        Eigen::VectorXd singularValues = svd.singularValues();

        double tolerance = 1e-9;
        int rank = (singularValues.array() > tolerance).count();

        Eigen::MatrixXd nullSpace = V.block(0, rank, V.rows(), V.cols() - rank);

        // null space vector의 선정 방법은 추후 확인 필요
        for (int i = 0; i < nullSpace.cols(); ++i)
        {
            Eigen::VectorXd vec = nullSpace.col(i);

            if ((vec.array() > 0).all())
            {
                return vec.normalized();
            }
        }

        throw std::runtime_error("No vector with all positive entries found in null space.");
    }

    void PropulsionControl::control_allocation()
    {
        Eigen::VectorXd thrust_star = pinv_allocation_matrix_ * wrench_;
        Eigen::VectorXd thrust_min = Eigen::VectorXd::Zero(8); // low thrust limit = 0N (음수 추력 X)
        Eigen::VectorXd delta_thrust = thrust_min - thrust_star;
        Eigen::VectorXd tmp = delta_thrust.array() / nullspace_vector_.array();

        double lambda = tmp.maxCoeff();

        thrust_ = pinv_allocation_matrix_ * wrench_ + lambda * nullspace_vector_; // SAM 방식

        for (int i = 0; i < 8; i++)
        {
            thrust_msg.data[i] = thrust_(i);
        }
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

    void PropulsionControl::thrust_publisher_callback()
    {
        if (kill_mode)
        {
            kill();
        }
        else
        {
            control_allocation();
            thrust_publisher_->publish(thrust_msg);

            Eigen::VectorXd generated_wrench = allocation_matrix_ * thrust_;
            for (int i = 0; i < 6; i++)
            {
                wrench_msg.data[i] = generated_wrench(i);
            }
        }
        wrench_publisher_->publish(wrench_msg);
    }

    void PropulsionControl::desired_wrench_callback(const std_msgs::msg::Float32MultiArray &msg)
    {
        wrench_(0) = msg.data[0]; // Fx
        wrench_(1) = msg.data[1]; // Fy
        wrench_(2) = msg.data[2]; // Fz
        wrench_(3) = msg.data[3]; // Tx
        wrench_(4) = msg.data[4]; // Ty
        wrench_(5) = msg.data[5]; // Tz
    }

    void PropulsionControl::kill()
    {
        thrust_ = Eigen::VectorXd::Zero(8);
    }

    void PropulsionControl::Arm()
    {
    }

    void PropulsionControl::disArm()
    {
    }
}