#include "rclcpp/rclcpp.hpp"
// #include "unitree_a1_legged_msgs/msg/high_cmd.hpp"
// #include "unitree_a1_legged_msgs/msg/high_state.hpp"
// #include "unitree_a1_legged_msgs/msg/low_cmd.hpp"
// #include "unitree_a1_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
// #include "unitree_a1_legged/convert.hpp"

using namespace UNITREE_LEGGED_SDK;

using namespace UNITREE_LEGGED_SDK;

class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::A1), udp(level){
        udp.InitCmdData(cmd_high);
        udp.InitCmdData(cmd_low);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();

    Safety safe;
    UDP udp;
    HighCmd cmd_high = {0};
    HighState state_high = {0};
    LowCmd cmd_low = {0};
    LowState state_low = {0};
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
};

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

int main(int argc, char **argv)
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    Custom custom(HIGHLEVEL);
    // // InitEnvironment();
    // LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    // LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    // LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    // loop_udpSend.start();
    // loop_udpRecv.start();
    // loop_control.start();

    while(1){
        sleep(10);
    };

    return 0; 
}