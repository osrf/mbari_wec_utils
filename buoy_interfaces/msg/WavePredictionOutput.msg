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

# Optional: higher-rate “actual at target” time series for frequency/phase comparison.
# This is typically populated from latent_data.inc_wave_heights[0] at the latent publish rate.
bool has_wec_actual_series
float64[] wec_series_time
float64[] wec_series_z
float64[] wec_series_u
float64[] wec_series_v

# Optional: dense model prediction at the target (WEC) over the measurement window.
# Useful for frequency/phase comparisons against `wec_series_*` without changing
# the future prediction grid resolution.
bool has_dense_predictions
float64[] dense_predictions_time
float64[] dense_predictions_z
float64[] dense_predictions_u
float64[] dense_predictions_v

# Solver diagnostics
float64 solve_time  # Time to solve least-squares problem [seconds]
int32 num_wavelengths  # Number of wave components solved
float64 centroid_period  # Spectral centroid period [seconds]

# Bulk wave parameters derived from the (directional) wavespec used for prediction.
# Direction convention matches SWIFTdirectionalspectra output used in the MATLAB example:
# nautical compass degrees True, direction waves are coming FROM.
bool has_wavespec_bulk
float64 wavespec_hs  # Significant wave height Hs [m]
float64 wavespec_tp  # Peak period Tp [s]
float64 wavespec_tm01  # Mean period Tm01 [s]
float64 wavespec_tm02  # Mean period Tm02 [s]
float64 wavespec_dp  # Peak direction Dp [deg True, FROM]
float64 wavespec_dm  # Mean direction Dm [deg True, FROM]
float64 wavespec_spreadp  # Directional spread at peak frequency [deg]
