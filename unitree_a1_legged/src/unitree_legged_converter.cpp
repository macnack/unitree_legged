#include "unitree_a1_legged/unitree_legged_converter.hpp"

using namespace UNITREE_LEGGED_SDK;

namespace unitree_legged
{
    sensor_msgs::msg::Imu Converter::stateToMsg(const IMU &state)
    {
        sensor_msgs::msg::Imu msg;
        msg.header.frame_id = "imu_link";
        msg.orientation.x = state.quaternion[0];
        msg.orientation.y = state.quaternion[1];
        msg.orientation.z = state.quaternion[2];
        msg.orientation.w = state.quaternion[3];
        msg.angular_velocity.x = state.gyroscope[0];
        msg.angular_velocity.y = state.gyroscope[0];
        msg.angular_velocity.z = state.gyroscope[0];
        msg.linear_acceleration.x = state.accelerometer[0];
        msg.linear_acceleration.y = state.accelerometer[1];
        msg.linear_acceleration.z = state.accelerometer[2];
        return msg;
    }
    unitree_a1_legged_msgs::msg::LowState Converter::stateToMsg(const LowState &state)
    {
        unitree_a1_legged_msgs::msg::LowState msg;
        msg.a1.level_flag = state.levelFlag;
        msg.a1.comm_version = state.commVersion;
        msg.a1.robot_id = state.robotID;
        msg.a1.sn = state.SN;
        msg.a1.band_width = state.bandWidth;
        msg.imu = Converter::stateToMsg(state.imu);
        for (int i = 0; i < 20; i++)
        {
            msg.motor_state[i] = Converter::stateToMsg(state.motorState[i]);
        }
        for (int i = 0; i < 4; i++)
        {
            msg.foot_force[i] = state.footForce[i];
            msg.foot_force_est[i] = state.footForceEst[i];
        }
        msg.tick = state.tick;
        for (int i = 0; i < 40; i++)
        {
            msg.wireless_remote[i] = state.wirelessRemote[i];
        }
        msg.reserve = state.reserve;
        msg.crc = state.crc;
        return msg;
    }
    unitree_a1_legged_msgs::msg::MotorState Converter::stateToMsg(const MotorState &state)
    {
        unitree_a1_legged_msgs::msg::MotorState msg;
        msg.mode = state.mode;
        msg.q = state.q;
        msg.dq = state.dq;
        msg.ddq = state.ddq;
        msg.tau_est = state.tauEst;
        msg.q_raw = state.q_raw;
        msg.dq_raw = state.dq_raw;
        msg.ddq_raw = state.ddq_raw;
        msg.temperature = state.temperature;
        msg.reserve[0] = state.reserve[0];
        msg.reserve[1] = state.reserve[1];
        return msg;
    }
    void Converter::msgToCmd(const unitree_a1_legged_msgs::msg::LowCmd::SharedPtr msg, LowCmd &cmd)
    {
        cmd.levelFlag = msg->a1.level_flag;
        cmd.commVersion = msg->a1.comm_version;
        cmd.robotID = msg->a1.robot_id;
        cmd.SN = msg->a1.sn;
        cmd.bandWidth = msg->a1.band_width;
        for (const auto &[key, value] : Converter::jointIndexMap)
        {
            cmd.motorCmd[value] = Converter::msgToCmd(msg->motor_cmd[value]);
        }
        for (int i = 0; i < 4; i++)
        {
            cmd.led[i].r = msg->led[i].r;
            cmd.led[i].g = msg->led[i].g;
            cmd.led[i].b = msg->led[i].b;
        }
        cmd.reserve = msg->reserve;
        cmd.crc = msg->crc;
    }
    void Converter::msgToCmd(const unitree_a1_legged_msgs::msg::QuadrupedCmd::SharedPtr msg, LowCmd &cmd)
    {
        cmd.motorCmd[FR_0] = Converter::msgToCmd(msg->front_right.hip);
        cmd.motorCmd[FR_1] = Converter::msgToCmd(msg->front_right.thigh);
        cmd.motorCmd[FR_2] = Converter::msgToCmd(msg->front_right.calf);
        cmd.motorCmd[FL_0] = Converter::msgToCmd(msg->front_left.hip);
        cmd.motorCmd[FL_1] = Converter::msgToCmd(msg->front_left.thigh);
        cmd.motorCmd[FL_2] = Converter::msgToCmd(msg->front_left.calf);
        cmd.motorCmd[RR_0] = Converter::msgToCmd(msg->rear_right.hip);
        cmd.motorCmd[RR_1] = Converter::msgToCmd(msg->rear_right.thigh);
        cmd.motorCmd[RR_2] = Converter::msgToCmd(msg->rear_right.calf);
        cmd.motorCmd[RL_0] = Converter::msgToCmd(msg->rear_left.hip);
        cmd.motorCmd[RL_1] = Converter::msgToCmd(msg->rear_left.thigh);
        cmd.motorCmd[RL_2] = Converter::msgToCmd(msg->rear_left.calf);
    }
    MotorCmd Converter::msgToCmd(const unitree_a1_legged_msgs::msg::MotorCmd &msg)
    {
        UNITREE_LEGGED_SDK::MotorCmd cmd;
        cmd.mode = msg.mode;
        cmd.q = msg.q;
        cmd.dq = msg.dq;
        cmd.tau = msg.tau;
        cmd.Kp = msg.kp;
        cmd.Kd = msg.kd;
        cmd.reserve[0] = msg.reserve[0];
        cmd.reserve[1] = msg.reserve[1];
        cmd.reserve[2] = msg.reserve[2];
        return cmd;
    }
    const std::unordered_map<std::string, int> Converter::jointIndexMap = {
        {"FR_hip_joint", FR_0},
        {"FR_thigh_joint", FR_1},
        {"FR_calf_joint", FR_2},
        {"FL_hip_joint", FL_0},
        {"FL_thigh_joint", FL_1},
        {"FL_calf_joint", FL_2},
        {"RR_hip_joint", RR_0},
        {"RR_thigh_joint", RR_1},
        {"RR_calf_joint", RR_2},
        {"RL_hip_joint", RL_0},
        {"RL_thigh_joint", RL_1},
        {"RL_calf_joint", RL_2}};

    std::vector<std::string> Converter::getJointNames()
    {
        std::vector<std::string> names;
        for (const auto &pair : Converter::jointIndexMap)
        {
            names.push_back(pair.first);
        }
        return names;
    }
    size_t Converter::getJointCount()
    {
        return Converter::jointIndexMap.size();
    }
    template <size_t N>
    static sensor_msgs::msg::JointState getJointStateMsg(const MotorState (&state)[N])
    {
        sensor_msgs::msg::JointState msg;
        msg.name = Converter::getJointNames();
        msg.position.resize(Converter::getJointCount());
        msg.velocity.resize(Converter::getJointCount());
        msg.effort.resize(Converter::getJointCount());
        for (const auto &[key, value] : Converter::jointIndexMap)
        {
            msg.position[value] = state[value].q;
            msg.velocity[value] = state[value].dq;
            msg.effort[value] = state[value].tauEst;
        }
        return msg;
    }

} // namespace unitree_legged