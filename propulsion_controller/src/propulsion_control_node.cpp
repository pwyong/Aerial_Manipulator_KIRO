#include "rclcpp/rclcpp.hpp"
#include "propulsion_controller/propulsion_control.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("propulsion_control_node");

    auto propulsion_control = std::make_shared<propulsion_controller::PropulsionControl>(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
