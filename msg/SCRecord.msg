#Spring Controller

std_msgs/Header header  # time
int16 seq_num
int16 load_cell
float32 range_finder
float32 upper_psi
float32 lower_psi
int64 epoch
float32 salinity
float32 temperature

uint8_t ReliefValveRequest=1  # Request to open/close valve
uint8_t ReliefValveStatus=128  # Status of Relief valve open/close
uint8_t PumpRequest=256  # Request to turn pump on or off
uint8_t PumpStatus=512  # Status of pump switch
uint8_t PumpOverTemp=1024  # Status of pump OverTemp signal
uint8_t PumpToggle=2048  # Status of pump Toggle.
uint8_t TetherPowerRequest=4096  # Request to turn tether power on or off
uint8_t TetherPowerStatus=8192  # Status of tether power relay
uint8_t LR_Fault=16384  # Status of LRF fault input
uint8_t AUX_Fault=32768  # Status of AUX fault input
uint16 status

