# relative_time is the time height was computed in decimal seconds
# relative to simulation time in pose header.stamp
float64 relative_time

# x, y is relative to buoy origin; otherwise world origin
bool use_buoy_origin

# header.stamp = simulation time
# position = x, y, z(height)
# orientation = normal vector (slope of wave) at position
geometry_msgs/PoseStamped pose
