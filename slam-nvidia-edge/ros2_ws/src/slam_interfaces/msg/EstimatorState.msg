# Full estimator output consumed by fusion_node / downstream planners.
std_msgs/Header header
geometry_msgs/Pose pose
geometry_msgs/Twist velocity
# Row-major 3x3 blocks: diag-style summary of position (m^2) and orientation
# (rad^2) marginal covariances. Placeholder until the estimator produces the
# full 6x6; kept fixed-size for deterministic serialization.
float64[9] covariance_position_orientation
string estimator_mode                  # INITIALIZING | TRACKING | DEGRADED | LOST
