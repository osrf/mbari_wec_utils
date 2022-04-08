#Trefoil Controller

std_msgs/Header header  # time
int32 seq_num
int32 power_timeouts
float32 tether_voltage
float32 battery_voltage
float32 pressure
float32[4] qtn
float32[3] mag  # gauss
int16 status  # bit field
float32[3] ang_rate  # rad/sec
int16 vpe_status  # bit field
float32[3] accel  # m/sec^2
int32 comms_timeouts
int32 motor_status  # Maxon status
int32 motor_current  # motor current
int32 encoder  # 32-bit encoder position

