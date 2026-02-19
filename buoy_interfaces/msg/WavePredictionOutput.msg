# Complete wave prediction output for a processing window
# Follows the least-squares wave propagation workflow

std_msgs/Header header  # Timestamp when prediction was computed

# Measurement window metadata
float64 window_start_time  # Time of first measurement in window (seconds, relative)
float64 window_end_time    # Time of last measurement in window (seconds, relative)
int32 n_measurements       # Number of measurement samples in window

# Wave spectra used for prediction
float64[] frequencies      # Wave frequencies [Hz]
float64[] directions       # Wave directions [degrees] (shape: n_directions)
float64[] energy_by_freq   # 1D spectrum energy [m^2/Hz]

# Predictions at target location (WEC buoy)
WavePredictionPoint[] predictions  # Array of predictions into future

# Reconstruction data (solution fit quality)
ReconstructionData reconstruction  # How well solution fits input measurements

# Solver diagnostics
float64 solve_time  # Time to solve least-squares problem [seconds]
int32 num_wavelengths  # Number of wave components solved
float64 centroid_period  # Spectral centroid period [seconds]
