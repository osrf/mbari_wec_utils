#Spring Controller

# All SI units
std_msgs/Header header  # time
int16 seq_num
int16 load_cell  # N
float32 range_finder  # m
float32 upper_pressure  # Pa
float32 lower_pressure  # Pa
int64 epoch
float32 salinity  # UNUSED
float32 temperature  # K, UNUSED

uint16 RELIEF_VALVE_REQUEST=1  # Request to open/close valve
uint16 RELIEF_VALVE_STATUS=128  # Status of Relief valve open/close
uint16 PUMP_REQUEST=256  # Request to turn pump on or off
uint16 PUMP_STATUS=512  # Status of pump switch
uint16 PUMP_OVER_TEMP=1024  # Status of pump OverTemp signal
uint16 PUMP_TOGGLE=2048  # Status of pump Toggle.
uint16 TETHER_POWER_REQUEST=4096  # Request to turn tether power on or off
uint16 TETHER_POWER_STATUS=8192  # Status of tether power relay
uint16 LR_FAULT=16384  # Status of LRF fault input
uint16 AUX_FAULT=32768  # Status of AUX fault input
uint16 status

