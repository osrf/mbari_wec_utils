# Power Controller

std_msgs/Header header  # time
int16 seq_num
float32 rpm
float32 sd_rpm
float32 voltage
float32 draw_curr_limit
float32 bcurrent
float32 wcurrent
float32 torque
float32 diff_press
float32 bias_current
float32 loaddc
float32 scale
float32 retract
float32 target_v
float32 target_a
float32 charge_curr_limit

# Note: Not Currently Implemented in Sim Controller
uint16 MODE=1  # controller mode. specified by defines in config.h (GENERATOR_MODE, TORQUE_MODE)
uint16 TORQUE_CMD_LIMITED=4  # defines what is limited the Torque Command
                           # (0 = Not limited, 1 = Rate, 2 = Min, 3 = Max).
uint16 BATT_SWITCH_REQUEST=8  # Indicates state of battery switch that the user desires.
                            # 0 = off, 1 = on
uint16 BATT_SWITCH_SETTING=16  # Indicates instantaneous setting of Battery Switch
uint16 SLOW_SWITCH_SETTING=32  # Indicates instantaneous setting of Bettery Slow Switch
uint16 EXT_VOLTAGE_DETECT=64  # Indicates if >170V is available on outside connector
uint16 GAIN_SCHEDULE_MODE=128  # 0 = off. 1 = on.
uint16 OVER_CURRENT_SHUTDOWN=256  # 1 indicates drive is in overcurrent shutdown.
uint16 OVER_VOLTAGE_SHUTDOWN=512  # 1 indicates drive is in overvoltage shutdown.
uint16 SPRING_RANGE_VALID=1024  # 1 indicates the current SC_Range value is valid (received within
                              # approximately SC_RANGE_VALID_TIMEOUT milliseconds)
uint16 PERMISSIVE_MODE=2048  # 1 indicates user has selected "permissive mode" which allows
                            # WindingCurrent commands to be set even if SpringController Range
                            # packets aren't arriving
uint16 status
