// #include "unitree_a1_legged/unitree_legged_node.hpp"
#include "unitree_a1_legged/unitree_legged_lcm.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_a1_legged/convert.hpp"


UnitreeLegged custom;

rclcpp::Publisher<unitree_a1_legged_msgs::msg::LowState>::SharedPtr pub_low;


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::cout << "v3 Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    

    // InitEnvironment();
    auto node = rclcpp::Node::make_shared("node_ros2_udp");
    pub_low = node->create_publisher<unitree_a1_legged_msgs::msg::LowState>("low_state", 1);

    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&UnitreeLegged::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&UnitreeLegged::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&UnitreeLegged::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    auto msg = unitree_a1_legged_msgs::msg::LowState();
    msg.a1.level_flag = 1;
    while(1){
        std::cout << "line" << std::endl;

        pub_low->publish(msg);
        std::cout << custom.state.imu.quaternion[2] << "\n\n\n" << std::endl;
        sleep(1);
    };
    rclcpp::shutdown();

    return 0; 
}
//colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On  -Wall -Wextra -Wpedantic
