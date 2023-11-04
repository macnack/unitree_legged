#include "unitree_a1_legged/convert.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"
using namespace UNITREE_LEGGED_SDK;

class UnitreeServer : public rclcpp::Node
{
public:
    UnitreeServer()
        : Node("unitree_lowlevel"), server_instance_(LOWLEVEL)
    {
        server_instance_.SubscribeState();
        motion_msg_.data = 0;
        pub_state_ = this->create_publisher<unitree_a1_legged_msgs::msg::LowState>("~/state", 1);
        pub_motion_time_ = this->create_publisher<std_msgs::msg::UInt32>("~/motiontime", 1);
        sub_command_ = this->create_subscription<unitree_a1_legged_msgs::msg::LowCmd>(
            "~/command", 1, std::bind(&UnitreeServer::receiveCommand, this, std::placeholders::_1));
        state_thread_ = std::thread(std::bind(&UnitreeServer::updateLoop, this));
    }
    ~UnitreeServer()
    {
        state_thread_.join();
    }

private:
    LCM server_instance_;
    LowCmd command_ = {0};
    LowState state_ = {0};
    std_msgs::msg::UInt32 motion_msg_;
    void receiveCommand(const unitree_a1_legged_msgs::msg::LowCmd::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Receiving: msg");
    }
    void updateLoop()
    {
        while (rclcpp::ok())
        {
            // server_instance_.Recv();
            // server_instance_.Get(state_);
            // auto state_msg = stateToRos(state_);
            // pub_state_->publish(state_msg);
            pub_motion_time_->publish(motion_msg_);
            motion_msg_.data = motion_msg_.data + 1;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    rclcpp::Publisher<unitree_a1_legged_msgs::msg::LowState>::SharedPtr pub_state_;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_motion_time_;
    rclcpp::Subscription<unitree_a1_legged_msgs::msg::LowCmd>::SharedPtr sub_command_;
    std::thread state_thread_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnitreeServer>());
    rclcpp::shutdown();
    return 0;
}