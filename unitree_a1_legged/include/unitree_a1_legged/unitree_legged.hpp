#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace UNITREE_LEGGED_SDK;

namespace unitree_legged
{

    class UnitreeLegged
    {
    public:
        UnitreeLegged();
        UDP getLowUdp();
        LowCmd getLowCmd();
        LowState getLowState();
        void sendLowCmd(LowCmd &cmd);
        void recvLowState();

    // private:
        UDP low_udp_;
        LowCmd low_cmd_ = {0};     // Warn: missing-field-initializers
        LowState low_state_ = {0}; // Warn: missing-field-initializers
    };

    class UnitreeLeggedHighLevel
    {
    public:
        UnitreeLeggedHighLevel();
        UDP getLowUdp();
        HighCmd getLowCmd();
        HighState getLowState();
        void sendLowCmd(const LowCmd &cmd);
        void recvLowState();

    private:
        UDP low_udp_;
        HighCmd low_cmd_ = {0};     // Warn: missing-field-initializers
        HighState low_state_ = {0}; // Warn: missing-field-initializers
    };

} // namespace unitree_legged