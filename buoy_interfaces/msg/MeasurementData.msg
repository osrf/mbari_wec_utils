# Raw measurements at instrument locations (used for plotting/debug)

# Time values for measurement samples (seconds, absolute time base)
float64[] time

# Instrument position (local coordinates in meters), shape: (N x B)
float64[] x_meas
float64[] y_meas

# Measured elevation and velocity at buoy locations, shape: (N x B)
float64[] z_meas
float64[] u_meas
float64[] v_meas

# Shape metadata for proper reshaping
int32 n_samples    # N (total measurement points)
int32 n_buoys      # B (number of instruments)
