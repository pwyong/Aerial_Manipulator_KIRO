#include "rclcpp/rclcpp.hpp"
#include "yaw_controller/yaw_control.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("yaw_control_node");

    auto winch_control = std::make_shared<yaw_controller::YawControl>(node);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
