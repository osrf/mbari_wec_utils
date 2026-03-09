# Time offset in seconds from pose.header.stamp
# sample_time = header.stamp + relative_time
float64 relative_time

# Currently always false; returned pose is in world coordinates
bool use_buoy_origin

# GPS reference for local Cartesian x/y in this sample
sensor_msgs/NavSatFix gps_ref

# header.stamp = simulation time at evaluation
# pose.position frame: world ENU (x=East, y=North, z=eta above waterplane)
# pose.orientation = surface normal at sample point
geometry_msgs/PoseStamped pose

# Surface velocities in world ENU:
# velocities.x/y = u/v (East/North), velocities.z = etadot
geometry_msgs/Vector3 velocities
