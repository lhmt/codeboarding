# Per-estimator health report; aggregated by health_monitor_node.
std_msgs/Header header
float64 tracking_quality               # 0 (lost) .. 1 (nominal)
int32 tracked_features
float64 reprojection_error_median      # px
float64 imu_gyro_bias_norm             # rad/s
float64 imu_accel_bias_norm            # m/s^2
float64 condition_number               # of the estimator information matrix
bool is_degenerate
bool is_lost
string status                          # OK | WARN_* | LOST | STALE (comma-joined)
