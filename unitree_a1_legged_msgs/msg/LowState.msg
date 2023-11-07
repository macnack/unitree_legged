std_msgs/msg/Header header
UnitreeHeader a1                   
sensor_msgs/Imu imu
MotorState[20] motor_state
int16[4] foot_force             # force sensors
int16[4] foot_force_est           # force sensors
uint32 tick                     # reference real-time from motion controller (unit: us)
uint8[40] wireless_remote        # wireless commands
uint32 reserve                  
uint32 crc    
