#include "unitree_a1_legged/unitree_legged_node.hpp"


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UnitreeLeggedNode>());
    rclcpp::shutdown();
    return 0;
}

// class Custom
// {
// public:
//     Custom(uint8_t level) : safe(LeggedType::A1), udp(level)
//     {
//         udp.InitCmdData(cmd);
//     }
//     void UDPRecv();
//     void UDPSend();
//     void RobotControl();

//     Safety safe;
//     UDP udp;
//     HighCmd cmd = {0};
//     HighState state = {0};
//     int motiontime = 0;
//     float dt = 0.002; // 0.001~0.01
// };

// void Custom::UDPRecv()
// {
//     udp.Recv();
// }

// void Custom::UDPSend()
// {
//     udp.Send();
// }

// void Custom::RobotControl()
// {
//     motiontime += 2;
//     udp.GetRecv(state);
//     printf("%d   %f\n", motiontime, state.imu.quaternion[2]);

//     cmd.forwardSpeed = 0.0f;
//     cmd.sideSpeed = 0.0f;
//     cmd.rotateSpeed = 0.0f;
//     cmd.bodyHeight = 0.0f;

//     cmd.mode = 0; // 0:idle, default stand      1:forced stand     2:walk continuously
//     cmd.roll = 0;
//     cmd.pitch = 0;
//     cmd.yaw = 0;

//     if (motiontime > 1000 && motiontime < 1500)
//     {
//         cmd.mode = 1;
//         cmd.roll = 0.5f;
//     }
//     if (motiontime > 20000)
//     {
//         cmd.mode = 1;
//     }

//     udp.SetSend(cmd);
// }

// rclcpp::Subscription<unitree_a1_legged_msgs::msg::HighCmd>::SharedPtr sub_high;
// rclcpp::Subscription<unitree_a1_legged_msgs::msg::LowCmd>::SharedPtr sub_low;

// rclcpp::Publisher<unitree_a1_legged_msgs::msg::HighState>::SharedPtr pub_high;
// rclcpp::Publisher<unitree_a1_legged_msgs::msg::LowState>::SharedPtr pub_low;

// long high_count = 0;
// long low_count = 0;

// void highCmdCallback(const unitree_a1_legged_msgs::msg::HighCmd::SharedPtr msg)
// {
//     printf("highCmdCallback is running !\t%ld\n", ::high_count);

//     // HighCmd state = {0};
//     // state = rosToCmd(msg);

//     // custom.udp.SetSend(state);
//     RCLCPP_INFO(rclint main(int argc, char *argv[])
// {
//     std::cout << "Communication level is set to HIGH-level." << std::endl
//               << "WARNING: Make sure the robot is standing on the ground." << std::endl
//               << "Press Enter to continue..." << std::endl;
//     std::cin.ignore();
//     Custom custom(HIGHLEVEL);

//     // UNITREE_LEGGED_SDK::UDP udp(0xff);
//     // rclcpp::init(argc, argv);
//     // auto node = rclcpp::Node::make_shared("node_ros2_udp");
//     // printf("high level runing!\n");
//     // pub_high = node->create_publisher<unitree_a1_legged_msgs::msg::HighState>("high_state", 1);
//     // sub_high = node->create_subscription<unitree_a1_legged_msgs::msg::HighCmd>("high_cmd", 1, highCmdCallback);
//     // rclcpp::spin(node);

//     // rclcpp::shutdown();
//     return 0;
// }cpp::get_logger("rclcpp"), "highCmdCallback is running !\t%ld\n", ::high_count);
//     printf("msg->head[0]: %d\n", msg->mode);
//     // printf("state.mode: %d\n", state.mode);
//     unitree_a1_legged_msgs::msg::HighState high_state_ros;

//     // high_state_ros = stateToRos(high_state);

//     pub_high->publish(high_state_ros);
//     printf("highCmdCallback ending !\t%ld\n\n", ::high_count++);
// }

// int main(int argc, char *argv[])
// {
//     std::cout << "Communication level is set to HIGH-level." << std::endl
//               << "WARNING: Make sure the robot is standing on the ground." << std::endl
//               << "Press Enter to continue..." << std::endl;
//     std::cin.ignore();
//     Custom custom(HIGHLEVEL);

//     // UNITREE_LEGGED_SDK::UDP udp(0xff);
//     // rclcpp::init(argc, argv);
//     // auto node = rclcpp::Node::make_shared("node_ros2_udp");
//     // printf("high level runing!\n");
//     // pub_high = node->create_publisher<unitree_a1_legged_msgs::msg::HighState>("high_state", 1);
//     // sub_high = node->create_subscription<unitree_a1_legged_msgs::msg::HighCmd>("high_cmd", 1, highCmdCallback);
//     // rclcpp::spin(node);

//     // rclcpp::shutdown();
//     return 0;
// }