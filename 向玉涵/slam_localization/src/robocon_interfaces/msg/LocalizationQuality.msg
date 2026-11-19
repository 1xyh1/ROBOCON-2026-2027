std_msgs/Header header
bool accepted
bool converged
float32 rmse_m
float32 overlap_ratio
float32 translation_step_m
float32 rotation_step_rad
# Ascending covariance eigenvalues [lambda_min, lambda_mid, lambda_max].
# The geometry gate uses lambda_min / lambda_max.
float64[3] geometry_eigenvalues
string floor
string source
string map_id
string calibration_id
uint64 odom_epoch
# Relocalization anchor identity. Continuous tracking retains this generation.
uint64 localization_generation
# Monotonic TF commit revision; advances for action AND continuous matches.
uint64 tf_revision
bool tf_committed
string rejection_reason

