#include "rclcpp/rclcpp.hpp"
#include "propulsion_controller/propulsion_control.hpp"

class PropulsionControlNode : public rclcpp::Node
{
public:
    PropulsionControlNode() : Node("propulsion_control")
    {
        propulsion_control_ = std::make_shared<platform_control::PropulsionControl>(this->shared_from_this());
    }

private:
    std::shared_ptr<platform_control::PropulsionControl> propulsion_control_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PropulsionControlNode>());
    rclcpp::shutdown();
    return 0;
}