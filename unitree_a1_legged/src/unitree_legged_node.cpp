#include "unitree_a1_legged/unitree_legged_node.hpp"

namespace unitree_legged

{

    UnitreeLeggedNode::UnitreeLeggedNode(const rclcpp::NodeOptions &options) : Node("unitree_lowlevel", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting Unitree LOWLEVEL Communication v2");
        pub_state_ = this->create_publisher<unitree_a1_legged_msgs::msg::LowState>("~/state", 1);
        sub_command_ = this->create_subscription<unitree_a1_legged_msgs::msg::LowCmd>("~/command", 1, std::bind(&UnitreeLeggedNode::receiveCommandCallback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(2ms, std::bind(&UnitreeLeggedNode::updateStateCallback, this));
        // state_thread_ = std::thread(std::bind(&UnitreeLeggedNode::updateLoop, this));
    }
    UnitreeLeggedNode::~UnitreeLeggedNode()
    {
        if (state_thread_.joinable())
            state_thread_.join();
    }

    void UnitreeLeggedNode::updateLoop()
    {
        while (rclcpp::ok())
        {
            unitree_a1_legged_msgs::msg::LowState low_state_ros;
            unitree.recvLowState();
            low_state_ros = Converter::stateToMsg(unitree.getLowState());
            pub_state_->publish(low_state_ros);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    void UnitreeLeggedNode::receiveCommandCallback(const unitree_a1_legged_msgs::msg::LowCmd::SharedPtr msg)
    {
        auto low_cmd_ = Converter::msgToCmd(msg);
        unitree.sendLowCmd(low_cmd_);
    }

    void UnitreeLeggedNode::updateStateCallback()
    {
        unitree_a1_legged_msgs::msg::LowState low_state_ros;
        unitree.recvLowState();
        low_state_ros = Converter::stateToMsg(unitree.getLowState());
        pub_state_->publish(low_state_ros);
    }

} // namespace unitree_legged