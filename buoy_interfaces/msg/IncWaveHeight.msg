# relative_time is the time height was computed in decimal seconds
# relative to simulation time in pose header.stamp
# time of wave height = header.stamp + relative_time
# Note: absolute_time from IncWaveHeight.srv converted to relative for response
# Note: all fixed-points in SDF are computed with relative_time = 0.0
float64 relative_time

# For now, position is always in world coords (use_buoy_origin always False)
bool use_buoy_origin

# header.stamp = simulation time of computation
#   time of wave height = header.stamp + relative_time
# position = x, y, z(height above waterplane)
# orientation = normal vector (slope of wave) at position
geometry_msgs/PoseStamped pose
