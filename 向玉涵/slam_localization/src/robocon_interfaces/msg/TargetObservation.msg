uint8 EARTH=1
uint8 SKY=2
uint8 STATUS_VALID=1
uint8 STATUS_AMBIGUOUS=2
uint8 STATUS_NO_DEPTH=3
uint8 STATUS_STALE=4

std_msgs/Header header
uint64 observation_id
string track_id
uint8 class_id
string visible_color
# Heuristic uncertainty for the current RGB-D prototype, not a calibrated
# measurement covariance; do not use it as an optimal fusion weight.
geometry_msgs/PoseWithCovariance object_pose
geometry_msgs/Vector3 dimensions
geometry_msgs/Point grasp_surface_center
geometry_msgs/Vector3 grasp_approach
float32 euclidean_range_m
float32 optical_depth_m
float32 depth_valid_ratio
float32 fit_residual_m
float32 confidence
bool orientation_observable
uint8 status
string calibration_version

