#Crossbow AHRS

std_msgs/Header header  # timestamp, seq_num

float32 roll_angle  # Radians -pi to +pi
float32 pitch_angle  # Radians -pi to +pi
float32 yaw_angle_true  # (true north) Radians -pi to +pi

float32 x_rate_corrected  # Corrected X angular rate, rads/sec
float32 y_rate_corrected  # Corrected Y angular rate, rads/sec
float32 z_rate_corrected  # Corrected Z angular rate, rads/sec

float32 x_accel  # X accelerometer, g
float32 y_accel  # Y accelerometer, g
float32 z_accel  # Z accelerometer, g

float32 n_velocity  # North velocity, m/s
float32 e_velocity  # East  velocity, m/s
float32 d_velocity  # Down  velocity, m/s

float32 longitude  # GPS longitude, radians
float32 latitude  # GPS latitude,  radians
float32 altitude  # GPS altitude,  meters (-100 to 16284)

float32 x_rate_temp  # X rate temperature, C

