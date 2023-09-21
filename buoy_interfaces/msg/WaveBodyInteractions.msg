
# Motion
geometry_msgs/Pose pose  # position/orientation
geometry_msgs/Twist twist  # linear/angular rates

# Forces/Torques
geometry_msgs/Wrench buoyancy
geometry_msgs/Wrench radiation
geometry_msgs/Wrench excitation

# Total power = dot(force, velocity) + dot(torque, omega)
float64 buoyancy_total_power  # Watts
float64 radiation_total_power  # Watts
float64 excitation_total_power  # Watts
