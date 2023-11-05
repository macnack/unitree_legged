#include "unitree_a1_legged/unitree_legged.hpp"

using namespace UNITREE_LEGGED_SDK;

namespace unitree_legged
{

    UnitreeLegged::UnitreeLegged()
        : low_udp_(LOWLEVEL)
    {
        low_udp_.InitCmdData(low_cmd_);
    }
    UDP UnitreeLegged::getLowUdp()
    {
        return low_udp_;
    }
    LowCmd UnitreeLegged::getLowCmd()
    {
        return low_cmd_;
    }
    LowState UnitreeLegged::getLowState()
    {
        return low_state_;
    }
    void UnitreeLegged::sendLowCmd(const LowCmd &cmd)
    {
        low_cmd_ = cmd;
        low_udp_.SetSend(low_cmd_);
        low_udp_.Send();
    }
    void UnitreeLegged::recvLowState()
    {
        low_udp_.Recv();
        low_udp_.GetRecv(low_state_);
    }

} // namespace unitree_legged