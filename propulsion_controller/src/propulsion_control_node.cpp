#include "rclcpp/rclcpp.hpp"
#include "propulsion_controller/propulsion_control.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Node 객체 생성
    auto node = std::make_shared<rclcpp::Node>("propulsion_control_node");

    // PropulsionControl 객체를 생성하여 Node에 종속시킴
    auto propulsion_control = std::make_shared<platform_control::PropulsionControl>(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
