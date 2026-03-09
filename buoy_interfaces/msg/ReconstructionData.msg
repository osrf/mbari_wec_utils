# Reconstructed measurements at instrument locations (fit diagnostics)

# Reconstructed sample times (seconds relative to window start), shape: (N)
float64[] time

# Reconstructed elevation at buoy locations (N x B)
float64[] z_recon

# Reconstructed east velocity at buoy locations (N x B)
float64[] u_recon

# Reconstructed north velocity at buoy locations (N x B)
float64[] v_recon

# Shape metadata for reshaping flat arrays
int32 n_samples    # N total measurement points
int32 n_buoys      # B instruments
