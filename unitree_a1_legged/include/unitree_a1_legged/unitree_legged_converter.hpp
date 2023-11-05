#include "unitree_a1_legged/unitree_legged.hpp"

#include "unitree_a1_legged_msgs/msg/low_cmd.hpp"
#include "unitree_a1_legged_msgs/msg/low_state.hpp"
#include "unitree_a1_legged_msgs/msg/motor_cmd.hpp"
#include "unitree_a1_legged_msgs/msg/motor_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace UNITREE_LEGGED_SDK;

namespace unitree_legged
{
    class Converter
    {
    public:
        static sensor_msgs::msg::Imu stateToMsg(const IMU &state);
        static unitree_a1_legged_msgs::msg::LowState stateToMsg(const LowState &state);
        static unitree_a1_legged_msgs::msg::MotorState stateToMsg(const MotorState &state);
        static LowCmd msgToCmd(const unitree_a1_legged_msgs::msg::LowCmd::SharedPtr msg);
        static MotorCmd msgToCmd(const unitree_a1_legged_msgs::msg::MotorCmd &msg);
    };

} // namespace unitree_legged