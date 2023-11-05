#include "rclcpp/rclcpp.hpp"
#include "unitree_a1_legged_msgs/msg/low_cmd.hpp"
#include "unitree_a1_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/quadruped.h"

using namespace UNITREE_LEGGED_SDK;

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos * (1 - rate) + targetPos * rate;
    return p;
}

class ExamplePosition : public rclcpp::Node
{
public:
    ExamplePosition() : Node("example_position")
    {
        state_ = this->create_subscription<unitree_a1_legged_msgs::msg::LowState>("unitree_lowlevel/state", 1, std::bind(&ExamplePosition::stateCallback, this, std::placeholders::_1));
        cmd_ = this->create_publisher<unitree_a1_legged_msgs::msg::LowCmd>("unitree_lowlevel/command", 1);
    }

private:
    int motiontime = 0;
    float qInit[3] = {0};
    float qDes[3] = {0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float Kp[3] = {0};
    float Kd[3] = {0};
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;
    rclcpp::Subscription<unitree_a1_legged_msgs::msg::LowState>::SharedPtr state_;
    rclcpp::Publisher<unitree_a1_legged_msgs::msg::LowCmd>::SharedPtr cmd_;
    void stateCallback(unitree_a1_legged_msgs::msg::LowState::SharedPtr msg)
    {
        motiontime++;
        unitree_a1_legged_msgs::msg::LowCmd cmd;

        cmd.motor_cmd[FR_0].tau = -0.65f;
        cmd.motor_cmd[FL_0].tau = +0.65f;
        cmd.motor_cmd[RR_0].tau = -0.65f;
        cmd.motor_cmd[RL_0].tau = +0.65f;

        if (motiontime >= 0)
        {
            // first, get record initial position
            // if( motiontime >= 100 && motiontime < 500){
            if (motiontime >= 0 && motiontime < 10)
            {
                qInit[0] = msg->motor_state[FR_0].q;
                qInit[1] = msg->motor_state[FR_1].q;
                qInit[2] = msg->motor_state[FR_2].q;
            }
            // second, move to the origin point of a sine movement with Kp Kd
            // if( motiontime >= 500 && motiontime < 1500){
            if (motiontime >= 10 && motiontime < 400)
            {
                rate_count++;
                double rate = rate_count / 200.0; // needs count to 200
                Kp[0] = 5.0;
                Kp[1] = 5.0;
                Kp[2] = 5.0;
                Kd[0] = 1.0;
                Kd[1] = 1.0;
                Kd[2] = 1.0;

                qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
                qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
                qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);
            }

            // last, do sine wave
            if (motiontime >= 400)
            {
                sin_count++;
                qDes[0] = sin_mid_q[0];
                qDes[1] = sin_mid_q[1];
                qDes[2] = sin_mid_q[2] -0.6 * sin(1.8 * M_PI * sin_count / 1000.0);
                // qDes[2] = sin_mid_q[2];
            }
        }
        cmd.motor_cmd[FR_0].q = qDes[0];
        cmd.motor_cmd[FR_0].dq = 0;
        cmd.motor_cmd[FR_0].kp = Kp[0];
        cmd.motor_cmd[FR_0].kd = Kd[0];
        cmd.motor_cmd[FR_0].tau = -0.65f;

        cmd.motor_cmd[FR_1].q = qDes[1];
        cmd.motor_cmd[FR_1].dq = 0;
        cmd.motor_cmd[FR_1].kp = Kp[1];
        cmd.motor_cmd[FR_1].kd = Kd[1];
        cmd.motor_cmd[FR_1].tau = 0.0f;

        cmd.motor_cmd[FR_2].q = qDes[2];
        cmd.motor_cmd[FR_2].dq = 0;
        cmd.motor_cmd[FR_2].kp = Kp[2];
        cmd.motor_cmd[FR_2].kd = Kd[2];
        cmd.motor_cmd[FR_2].tau = 0.0f;
        cmd_->publish(cmd);
    }
};
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExamplePosition>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}