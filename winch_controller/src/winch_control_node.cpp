#include "rclcpp/rclcpp.hpp"
#include "winch_controller/winch_control.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("winch_control_node");

    auto winch_control = std::make_shared<winch_controller::WinchControl>(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
