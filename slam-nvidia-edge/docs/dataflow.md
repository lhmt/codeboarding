# Dataflow: topics and node graph

## Inputs (drivers, not part of this repo)

| Topic | Type | Source |
|---|---|---|
| `/camera/image` | `sensor_msgs/Image` | camera driver (Argus / v4l2) |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | camera driver |
| `/imu/data` | `sensor_msgs/Imu` | IMU driver |
| `/lidar/points` | `sensor_msgs/PointCloud2` | LiDAR driver |
| `/gps/fix` (optional) | `sensor_msgs/NavSatFix` | GNSS driver |
| `/wheel/odom` (optional) | `nav_msgs/Odometry` | base controller |

## SLAM topics

| Topic | Type | Publisher |
|---|---|---|
| `/slam/frontend/features` | `sensor_msgs/PointCloud2` | tensor_frontend_node |
| `/slam/vio/pose` | `geometry_msgs/PoseStamped` | vio_node |
| `/slam/vio/state` | `slam_interfaces/EstimatorState` | vio_node |
| `/slam/vio/health` | `slam_interfaces/PoseHealth` | vio_node |
| `/slam/lio/pose` | `geometry_msgs/PoseStamped` | lio_node |
| `/slam/lio/state` | `slam_interfaces/EstimatorState` | lio_node |
| `/slam/lio/health` | `slam_interfaces/PoseHealth` | lio_node |
| `/slam/lio/local_map` | `sensor_msgs/PointCloud2` | lio_node |
| `/slam/fused/pose` | `geometry_msgs/PoseStamped` | fusion_node |
| `/slam/fused/state` | `slam_interfaces/EstimatorState` | fusion_node |
| `/slam/fused/health` | `slam_interfaces/PoseHealth` | fusion_node |
| `/slam/map/points` | `sensor_msgs/PointCloud2` | map_node |
| `/slam/health/global` | `slam_interfaces/PoseHealth` | health_monitor_node |
| `/slam/sync/status` | `slam_interfaces/SensorSyncStatus` | vio_node |

## Node graph

```
/camera/image ────► tensor_frontend_node ──/slam/frontend/features──► (vio_node, optional)
/camera/image ────► vio_node
/camera/camera_info ► vio_node
/imu/data ─────┬──► vio_node ──► /slam/vio/{pose,state,health}
               └──► lio_node ──► /slam/lio/{pose,state,health,local_map}
/lidar/points ────► lio_node
/slam/vio/state ──► fusion_node ─► /slam/fused/{pose,state,health}
/slam/lio/state ──► fusion_node
/gps/fix ─────────► fusion_node        (optional, param-gated)
/wheel/odom ──────► fusion_node        (optional, param-gated)
/slam/lio/local_map ► map_node ──────► /slam/map/points
/slam/*/health ───► health_monitor_node ► /slam/health/global
/slam/fused/state, /slam/*/health, /camera/camera_info ► recorder_node ► logs/
```

## QoS

- Sensor inputs: `rclcpp::SensorDataQoS()` (best-effort, small queue).
- Estimator outputs (`pose`, `state`, `health`): reliable, depth 10.
- Local map / map points: reliable, depth 1 (latest wins for large payloads).
