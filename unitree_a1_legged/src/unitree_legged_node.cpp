#include "unitree_a1_legged/unitree_legged_node.hpp"

namespace unitree_legged

{

    UnitreeLeggedNode::UnitreeLeggedNode(const rclcpp::NodeOptions &options) : Node("unitree_lowlevel", options)
    {
        RCLCPP_INFO(this->get_logger(), "Starting Unitree LOWLEVEL Communication v2");

        const size_t num_joints = Converter::getJointNames().size();
        joint_state_msg_.name = Converter::getJointNames();
        joint_state_msg_.position.resize(num_joints);
        joint_state_msg_.velocity.resize(num_joints);
        joint_state_msg_.effort.resize(num_joints);
        joint_state_msg_.header.frame_id = "base_link";

        pub_state_ = this->create_publisher<unitree_a1_legged_msgs::msg::LowState>("~/state", 1);
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 1);
        sub_command_ = this->create_subscription<unitree_a1_legged_msgs::msg::LowCmd>("~/low_command", 1, std::bind(&UnitreeLeggedNode::receiveCommandCallback, this, std::placeholders::_1));
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
        auto low_cmd_once = unitree.getLowCmd();
        Converter::msgToCmd(msg, low_cmd_once);
        unitree.sendLowCmd(low_cmd_once);
    }

    void UnitreeLeggedNode::updateStateCallback()
    {
        unitree_a1_legged_msgs::msg::LowState low_state_ros;
        unitree.recvLowState();
        low_state_ros = Converter::stateToMsg(unitree.getLowState());
        pub_state_->publish(low_state_ros);
        joint_state_msg_.header.stamp = this->now();
        for (const auto &[key, value] : Converter::jointIndexMap)
            joint_state_msg_.position[value] = unitree.getLowState().motorState[value].q;
        joint_state_publisher_->publish(joint_state_msg_);
    }

} // namespace unitree_legged