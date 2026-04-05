# ground_projection

Projects image-space line segments from `line_detector` onto the ground plane using a homography computed from the camera's extrinsic calibration. The projected segments are in the robot's body frame (meters) and feed into `lane_filter`.

## Scripts

| Script | Description |
|--------|-------------|
| `scripts/ground_projection.py` | Main projection node |
| `scripts/ground_projection_node.py` | ROS node wrapper |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~lineseglist_in` | `rosky_msgs/SegmentList` | Detected segments in image coordinates |
| `~camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~lineseglist_out` | `rosky_msgs/SegmentList` | Segments projected to ground plane (meters) |

## Calibration

The homography is loaded from `rosky_base/config/baseline/calibration/camera_extrinsic/<hostname>.yaml`. Run extrinsic calibration by placing ROSKY on the calibration pattern (`setup/calibration_pattern_ROSKY.pdf`) and using the intrinsic calibration launch:

```bash
roslaunch rosky_base intrinsic_calibration.launch veh:=rosky01
```

The `homography/` directory contains helper scripts for manually computing or verifying the homography matrix.
