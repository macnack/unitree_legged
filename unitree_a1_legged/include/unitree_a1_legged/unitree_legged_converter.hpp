#include <unordered_map>
#include "unitree_a1_legged/unitree_legged.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "unitree_a1_legged_msgs/msg/low_cmd.hpp"
#include "unitree_a1_legged_msgs/msg/low_state.hpp"
#include "unitree_a1_legged_msgs/msg/motor_cmd.hpp"
#include "unitree_a1_legged_msgs/msg/motor_state.hpp"
#include "unitree_a1_legged_msgs/msg/quadruped_cmd.hpp"


using namespace UNITREE_LEGGED_SDK;

namespace unitree_legged
{
    class Converter
    {
    public:
        static sensor_msgs::msg::Imu stateToMsg(const IMU &state);
        static unitree_a1_legged_msgs::msg::LowState stateToMsg(const LowState &state);
        static unitree_a1_legged_msgs::msg::MotorState stateToMsg(const MotorState &state);
        static void msgToCmd(const unitree_a1_legged_msgs::msg::LowCmd::SharedPtr msg , LowCmd &cmd);
        static void msgToCmd(const unitree_a1_legged_msgs::msg::QuadrupedCmd::SharedPtr msg , LowCmd &cmd);
        static MotorCmd msgToCmd(const unitree_a1_legged_msgs::msg::MotorCmd &msg);
        static const std::unordered_map<std::string, int> jointIndexMap;
        static std::vector<std::string> getJointNames();
        static size_t getJointCount();
        template <size_t N>
        static sensor_msgs::msg::JointState getJointStateMsg(const MotorState (&state)[N]);
    };

} // namespace unitree_legged