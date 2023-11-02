#include "unitree_a1_legged/unitree_legged_node.hpp"


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnitreeLeggedNode>());
    rclcpp::shutdown();
    return 0;
}
