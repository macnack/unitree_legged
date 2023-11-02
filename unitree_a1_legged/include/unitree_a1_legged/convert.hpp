#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_a1_legged_msgs/msg/sphere.hpp"
#include "unitree_a1_legged_msgs/msg/high_cmd.hpp"
#include "unitree_a1_legged_msgs/msg/high_state.hpp"
#include "unitree_a1_legged_msgs/msg/low_cmd.hpp"
#include "unitree_a1_legged_msgs/msg/low_state.hpp"
#include "unitree_a1_legged_msgs/msg/motor_cmd.hpp"
#include "unitree_a1_legged_msgs/msg/motor_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

unitree_a1_legged_msgs::msg::Sphere convert()
{
    unitree_a1_legged_msgs::msg::Sphere sphere;
    sphere.radius = 0.44;
    return sphere;
}

UNITREE_LEGGED_SDK::HighCmd rosToCmd(const unitree_a1_legged_msgs::msg::HighCmd::SharedPtr msg);
unitree_a1_legged_msgs::msg::HighState stateToRos(const UNITREE_LEGGED_SDK::HighState &state);

UNITREE_LEGGED_SDK::LowCmd rosToCmd(const unitree_a1_legged_msgs::msg::LowCmd::SharedPtr msg);
unitree_a1_legged_msgs::msg::LowState stateToRos(const UNITREE_LEGGED_SDK::LowState &state);

UNITREE_LEGGED_SDK::MotorCmd rosMotorToCmd(const unitree_a1_legged_msgs::msg::MotorCmd &msg);
unitree_a1_legged_msgs::msg::MotorState stateToRos(const UNITREE_LEGGED_SDK::MotorState &state);

UNITREE_LEGGED_SDK::HighCmd rosToCmd(const unitree_a1_legged_msgs::msg::HighCmd::SharedPtr msg)
{
    UNITREE_LEGGED_SDK::HighCmd state;
    state.levelFlag = msg->a1.level_flag;
    state.commVersion = msg->a1.comm_version;
    state.robotID = msg->a1.robot_id;
    state.SN = msg->a1.sn;
    state.bandWidth = msg->a1.band_width;
    state.mode = msg->mode;
    state.forwardSpeed = msg->forward_speed;
    state.sideSpeed = msg->side_speed;
    state.rotateSpeed = msg->rotate_speed;
    state.bodyHeight = msg->body_height;
    state.footRaiseHeight = msg->foot_raise_height;
    state.yaw = msg->yaw;
    state.pitch = msg->pitch;
    state.roll = msg->roll;
    for (int i = 0; i < 4; i++)
    {
        state.led[i].r = msg->led[i].r;
        state.led[i].g = msg->led[i].g;
        state.led[i].b = msg->led[i].b;
    }
    for (int i = 0; i < 40; i++)
    {
        state.wirelessRemote[i] = msg->wireless_remote[i];
        state.AppRemote[i] = msg->app_remote[i];
    }
    state.reserve = msg->reserve;
    state.crc = msg->crc;
    return state;
}

unitree_a1_legged_msgs::msg::HighState stateToRos(const UNITREE_LEGGED_SDK::HighState &state)
{
    unitree_a1_legged_msgs::msg::HighState msg;
    msg.a1.level_flag = state.levelFlag;
    msg.a1.comm_version = state.commVersion;
    msg.a1.robot_id = state.robotID;
    msg.a1.sn = state.SN;
    msg.a1.band_width = state.bandWidth;
    msg.mode = state.mode;
    msg.forward_speed = state.forwardSpeed;
    msg.side_speed = state.sideSpeed;
    msg.rotate_speed = state.rotateSpeed;
    msg.body_height = state.bodyHeight;
    msg.up_down_speed = state.updownSpeed;
    msg.forward_position = state.forwardPosition;
    msg.side_position = state.sidePosition;
    for (int i = 4; i < 4; i++)
    {
        msg.foot_position_to_body[i].x = state.footPosition2Body[i].x;
        msg.foot_position_to_body[i].y = state.footPosition2Body[i].x;
        msg.foot_position_to_body[i].z = state.footPosition2Body[i].x;
        msg.foot_speed_to_body[i].x = state.footSpeed2Body[i].x;
        msg.foot_speed_to_body[i].y = state.footSpeed2Body[i].y;
        msg.foot_speed_to_body[i].z = state.footSpeed2Body[i].z;
        msg.foot_force[i] = state.footForce[i];
        msg.foot_force_est[i] = state.footForceEst[i];
    }
    // Imu msg:
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.frame_id = "imu_link";
    imu_msg.orientation.x = state.imu.quaternion[0];
    imu_msg.orientation.y = state.imu.quaternion[1];
    imu_msg.orientation.z = state.imu.quaternion[2];
    imu_msg.orientation.w = state.imu.quaternion[3];
    imu_msg.angular_velocity.x = state.imu.gyroscope[0];
    imu_msg.angular_velocity.y = state.imu.gyroscope[0];
    imu_msg.angular_velocity.z = state.imu.gyroscope[0];
    imu_msg.linear_acceleration.x = state.imu.accelerometer[0];
    imu_msg.linear_acceleration.y = state.imu.accelerometer[1];
    imu_msg.linear_acceleration.z = state.imu.accelerometer[2];

    msg.imu = imu_msg;
    msg.tick = state.tick;
    for (int i = 0; i < 40; i++)
    {
        msg.wireless_remote[i] = state.wirelessRemote[i];
    }
    msg.reserve = state.reserve;
    msg.crc = state.crc;
    return msg;
}

UNITREE_LEGGED_SDK::LowCmd rosToCmd(const unitree_a1_legged_msgs::msg::LowCmd::SharedPtr msg)
{
    UNITREE_LEGGED_SDK::LowCmd cmd;
    cmd.levelFlag = msg->a1.level_flag;
    cmd.commVersion = msg->a1.comm_version;
    cmd.robotID = msg->a1.robot_id;
    cmd.SN = msg->a1.sn;
    cmd.bandWidth = msg->a1.band_width;
    for (int i = 0; i < 20; i++)
    {
        cmd.motorCmd[i] = rosMotorToCmd(msg->motor_cmd[i]);
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

unitree_a1_legged_msgs::msg::LowState stateToRos(const UNITREE_LEGGED_SDK::LowState &state)
{
    unitree_a1_legged_msgs::msg::LowState msg;
    msg.a1.level_flag = state.levelFlag;
    msg.a1.comm_version = state.commVersion;
    msg.a1.robot_id = state.robotID;
    msg.a1.sn = state.SN;
    msg.a1.band_width = state.bandWidth;
    for (int i = 0; i < 20; i++)
    {
        msg.motor_state[i] = stateToRos(state.motorState[i]);
    }
    for (int i = 0; i < 4; i++)
    {
        msg.foot_force[i] = state.footForce[i];
        msg.foot_force_est[i] = state.footForceEst[i];
    }
    msg.reserve = state.reserve;
    msg.crc = state.crc;
    return msg;
}

UNITREE_LEGGED_SDK::MotorCmd rosMotorToCmd(const unitree_a1_legged_msgs::msg::MotorCmd &msg)
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

unitree_a1_legged_msgs::msg::MotorState stateToRos(const UNITREE_LEGGED_SDK::MotorState &state)
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
