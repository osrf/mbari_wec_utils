# Prediction sample at one time/location

std_msgs/Header header  # Absolute timestamp for this sample (ROS/sim time)

# Prediction time in seconds (absolute; same base as header.stamp)
float64 time

# Prediction location in local coordinates (meters)
float64 x  # easting from WEC buoy
float64 y  # northing from WEC buoy

# Predicted sea surface elevation (meters)
float64 elevation

# Predicted velocity components in local ENU (m/s)
float64 vel_east
float64 vel_north
