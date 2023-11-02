#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "unitree_a1_legged/convert.hpp"

using namespace UNITREE_LEGGED_SDK;
using namespace std::chrono_literals;


class UnitreeLeggedNode : public rclcpp::Node
{
public:
    UnitreeLeggedNode()
        : Node("unitree_legged"), udp_(LOWLEVEL)
    {
        udp_.InitCmdData(cmd_);
        this->declare_parameter("period_ms", 100);
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

private:
    void timer_callback()
    {
        receiveState();
        auto msg = stateToRos(state_);
        pub_low_->publish(msg);
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! ";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    UDP udp_;
    LowCmd cmd_ = {0};
    LowState state_ = {0};
    void receiveState()
    {
        udp_.Recv();
        udp_.GetRecv(state_);
    }
    void sendCommand()
    {
        udp_.SetSend(cmd_);
        udp_.Send();
    }
    void receiveCommand(const unitree_a1_legged_msgs::msg::LowCmd::SharedPtr msg)
    {
        cmd_ = rosToCmd(msg);
        sendCommand();
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<unitree_a1_legged_msgs::msg::LowState>::SharedPtr pub_low_;
    rclcpp::Subscription<unitree_a1_legged_msgs::msg::LowCmd>::SharedPtr sub_low_;
};