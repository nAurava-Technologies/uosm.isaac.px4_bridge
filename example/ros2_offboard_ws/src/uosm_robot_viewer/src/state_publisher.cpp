#include "rclcpp/rclcpp.hpp"

class StatePublisher : public rclcpp::Node
{
public:
    StatePublisher() : Node("state_publisher")
    {
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StatePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}