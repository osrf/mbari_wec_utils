# Reconstructed measurements at instrument locations (validation of solver fit)
# Shows how well the solution explains the input data

# Time values for reconstructed samples (seconds relative to start of window, shape: N)
float64[] time

# Reconstructed elevation at 4 SWIFT buoy locations (N x 4)
float64[] z_recon

# Reconstructed east velocity at 4 SWIFT buoy locations (N x 4)
float64[] u_recon

# Reconstructed north velocity at 4 SWIFT buoy locations (N x 4)
float64[] v_recon

# Shape metadata for proper reshaping
int32 n_samples    # N (total measurement points)
int32 n_buoys      # 4 (number of instruments)
