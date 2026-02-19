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

# Target location in local coordinates (meters)
float64 x_target
float64 y_target

# Raw measurements used for the solve (for plotting/debug)
MeasurementData measurements

# Predictions at target location (WEC buoy)
WavePredictionPoint[] predictions  # Array of predictions into future

# Reconstruction data (solution fit quality)
ReconstructionData reconstruction  # How well solution fits input measurements

# Optional: latest "actual at target" sample packaged alongside the prediction.
# This is commonly sourced from latent_data.inc_wave_heights[0].
bool has_wec_actual
float64 wec_time
float64 wec_z
float64 wec_u
float64 wec_v

# Solver diagnostics
float64 solve_time  # Time to solve least-squares problem [seconds]
int32 num_wavelengths  # Number of wave components solved
float64 centroid_period  # Spectral centroid period [seconds]
