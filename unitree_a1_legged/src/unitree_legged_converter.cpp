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
    LowCmd Converter::msgToCmd(const unitree_a1_legged_msgs::msg::LowCmd::SharedPtr msg)
    {
        LowCmd cmd;
        cmd.levelFlag = msg->a1.level_flag;
        cmd.commVersion = msg->a1.comm_version;
        cmd.robotID = msg->a1.robot_id;
        cmd.SN = msg->a1.sn;
        cmd.bandWidth = msg->a1.band_width;
        for (int i = 0; i < 20; i++)
        {
            cmd.motorCmd[i] = Converter::msgToCmd(msg->motor_cmd[i]);
        }
        for (int i = 0; i < 4; i++)
        {
            cmd.led[i].r = msg->led[i].r;
            cmd.led[i].g = msg->led[i].g;
            cmd.led[i].b = msg->led[i].b;
        }
        cmd.reserve = msg->reserve;
        cmd.crc = msg->crc;
        return cmd;
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

} // namespace unitree_legged