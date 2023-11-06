
#include "unitree_a1_legged/unitree_legged_converter.hpp"

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

using namespace UNITREE_LEGGED_SDK;

namespace unitree_legged

{
    class UnitreeLeggedNode : public rclcpp::Node
    {
    public:
        explicit UnitreeLeggedNode(const rclcpp::NodeOptions & options);
        ~UnitreeLeggedNode();

    private:
        std::thread state_thread_;
        UnitreeLegged unitree;
        sensor_msgs::msg::JointState joint_state_msg_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<unitree_a1_legged_msgs::msg::LowCmd>::SharedPtr sub_command_;
        rclcpp::Publisher<unitree_a1_legged_msgs::msg::LowState>::SharedPtr pub_state_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
        void updateLoop();
        void receiveCommandCallback(const unitree_a1_legged_msgs::msg::LowCmd::SharedPtr msg);
        void updateStateCallback();
        bool switch_once = true;
    };
} // namespace unitree_legged