#Trefoil Controller

std_msgs/Header header  # time
int32 seq_num
int32 power_timeouts
float32 tether_voltage
float32 battery_voltage
float32 pressure

# Quaternion in meters
geometry_msgs/Quaternion qtn

# Magnetic field in Tesla
sensor_msgs/MagneticField mag

int16 status  # bit field

# Angular velocity in rad/s
geometry_msgs/Vector3 ang_rate

int16 vpe_status  # bit field

# Linear acceleration m/s^2
geometry_msgs/Vector3 accel

int32 comms_timeouts
int32 motor_status  # Maxon status
int32 motor_current  # motor current
int32 encoder  # 32-bit encoder position

