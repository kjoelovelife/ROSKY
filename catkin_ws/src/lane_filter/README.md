# lane_filter

Estimates the robot's position within the lane using a Kalman filter. Takes ground-projected line segments from `ground_projection` and outputs a `LanePose` message with lateral offset `d` (meters from lane center) and heading error `phi` (radians).

## Nodes

| Node | Script | Description |
|------|--------|-------------|
| `lane_filter_node` | `src/lane_filter_node.py` | Primary Kalman filter (white + yellow lines) |
| `lane_filter_node_Yellow` | `src/lane_filter_node_Yellow.py` | Yellow-line-only variant |
| `lane_filter_node_inverse` | `src/lane_filter_node_inverse.py` | Inverse lane (right-side driving) variant |
| `lane_pose_visualizer_node` | `src/lane_pose_visualizer_node.py` | Publishes a debug image showing lane pose estimate |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~segment_list` | `rosky_msgs/SegmentList` | Ground-projected segments from `ground_projection` |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~lane_pose` | `rosky_msgs/LanePose` | Estimated lateral offset `d` and heading error `phi` |
| `~belief_img` | `sensor_msgs/Image` | Debug visualization of the filter belief distribution |

## Filter Details

The filter maintains a 2D belief over `(d, phi)` using a histogram/particle approach seeded by a Kalman prediction step. Line segments are scored against expected line positions for each hypothesis, and the belief is updated accordingly. Filter parameters (grid resolution, noise covariances, initial state) are configurable via `dynamic_reconfigure`.
