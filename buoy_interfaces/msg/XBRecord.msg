# Crossbow AHRS

# Timestamp, seq_num
std_msgs/Header header

# IMU data
# * orientation expressed as a quaternion (radians)
# * angular velocity (rad/s)
# * linear acceleration (m/s^2, not in g)
sensor_msgs/Imu imu

# GPS latitude, longitude, altitude
sensor_msgs/NavSatFix gps

# Linear velocity expressed in North-East-Down frame, in m/s
geometry_msgs/Vector3 ned_velocity

# X rate temperature, C
sensor_msgs/Temperature x_rate_temp
