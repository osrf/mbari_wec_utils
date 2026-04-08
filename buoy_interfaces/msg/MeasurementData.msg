# Raw measurements at instrument locations (plot/debug support)

# Measurement times (seconds, absolute time base)
float64[] time

# Instrument positions in local coordinates (meters), shape: (N x B)
float64[] x_meas
float64[] y_meas

# Measured elevation and velocity at buoy locations, shape: (N x B)
float64[] z_meas
float64[] u_meas
float64[] v_meas

# Shape metadata for reshaping flat arrays
int32 n_samples    # N total measurement points
int32 n_buoys      # B instruments
