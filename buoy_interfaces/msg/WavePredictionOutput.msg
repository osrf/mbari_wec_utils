# Wave prediction output for one processing window

std_msgs/Header header  # Timestamp for this output message

# Measurement window metadata
float64 window_start_time  # Time of first measurement in window (seconds, relative)
float64 window_end_time    # Time of last measurement in window (seconds, relative)
int32 n_measurements       # Number of measurement samples in window

# Wavespec used for prediction
float64[] frequencies      # Wave frequencies [Hz]
float64[] directions       # Wave directions [deg True, FROM] (0°=North, 90°=East), shape: n_directions
float64[] energy_by_freq   # 1D spectrum energy [m^2/Hz]

# Target location in local coordinates (meters)
float64 x_target
float64 y_target

# Input measurements used by solver (plot/debug)
MeasurementData measurements

# Future predictions at target location
WavePredictionPoint[] predictions

# Reconstruction at measurement locations (fit quality)
ReconstructionData reconstruction

# Optional latest "actual at target" sample
bool has_wec_actual
float64 wec_time
float64 wec_z
float64 wec_u
float64 wec_v

# Optional higher-rate "actual at target" series
bool has_wec_actual_series
float64[] wec_series_time
float64[] wec_series_z
float64[] wec_series_u
float64[] wec_series_v

# Optional dense model prediction at target over the measurement window
# Useful for frequency/phase comparison against wec_series_*.
bool has_dense_predictions
float64[] dense_predictions_time
float64[] dense_predictions_z
float64[] dense_predictions_u
float64[] dense_predictions_v

# Solver diagnostics
float64 solve_time  # Time to solve least-squares problem [seconds]
int32 num_wavelengths  # Number of wave components solved
float64 centroid_period  # Spectral centroid period [seconds]

# Bulk wavespec parameters (directions in deg True, FROM)
bool has_wavespec_bulk
float64 wavespec_hs  # Significant wave height Hs [m]
float64 wavespec_tp  # Peak period Tp [s]
float64 wavespec_tm01  # Mean period Tm01 [s]
float64 wavespec_tm02  # Mean period Tm02 [s]
float64 wavespec_dp  # Peak direction Dp [deg True, FROM]
float64 wavespec_dm  # Mean direction Dm [deg True, FROM]
float64 wavespec_spreadp  # Directional spread at peak frequency [deg]
