#include "rclcpp/rclcpp.hpp"
#include "oscillation_damping_controller/oscillation_damping_control.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("oscillation_damping_control_node");

    auto winch_control = std::make_shared<oscillation_damping_controller::DampingControl>(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
