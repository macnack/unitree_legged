#include "unitree_a1_legged/unitree_legged_node.hpp"

// UnitreeLeggedNodeA::UnitreeLeggedNodeA(const rclcpp::NodeOptions &options) : Node("unitree_legged", options)
UnitreeLeggedNodeA::UnitreeLeggedNodeA() : Node("unitree_legged")
{
    udp_.InitCmdData(cmd_);
    this->declare_parameter("period_ms", 500);
    int period_ms;
    this->get_parameter("period_ms", period_ms);
    std::chrono::duration<int64_t, std::milli> period = std::chrono::duration<int64_t, std::milli>(period_ms);

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    pub_low_ = this->create_publisher<unitree_a1_legged_msgs::msg::LowState>("low_state", 10);
    sub_low_ = this->create_subscription<unitree_a1_legged_msgs::msg::LowCmd>(
        "topic_new", 10, std::bind(&UnitreeLeggedNode::receiveCommand, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        period, std::bind(&UnitreeLeggedNode::timer_callback, this));
}

void UnitreeLeggedNodeA::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! ";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}

void UnitreeLeggedNodeA::receiveState()
{
    udp_.Recv();
    udp_.GetRecv(state_);
}

void UnitreeLeggedNodeA::sendCommand()
{
    udp_.SetSend(cmd_);
    udp_.Send();
}

void UnitreeLeggedNodeA::receiveCommand(const unitree_a1_legged_msgs::msg::LowCmd::SharedPtr msg)
{
    cmd_ = rosMsg2Cmd(msg);
    this->sendCommand();
}
