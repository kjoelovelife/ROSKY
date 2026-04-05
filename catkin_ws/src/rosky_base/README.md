# rosky_base

Meta-package that holds all launch files, configuration files, and calibration data for ROSKY. It has no executable nodes of its own.

## Launch Files

| Launch File | Description |
|-------------|-------------|
| `lane_following.launch` | Starts the full lane-following stack (camera → perception → control → motor) |
| `teleop_keyboard.launch` | Manual keyboard control |
| `intrinsic_calibration.launch` | Camera intrinsic calibration (checkerboard) |

Pass the robot name with `veh:=<name>` (default: `rosky01`):

```bash
roslaunch rosky_base lane_following.launch veh:=rosky01
```

## Configuration

Per-robot YAML files live under `config/baseline/`. The script `~/ROSKY/setup/set_package_param.sh` copies `default.yaml` to `<hostname>.yaml` for each config group on first run.

Key configuration groups:

| Directory | Contents |
|-----------|----------|
| `calibration/camera_intrinsic/` | Camera matrix and distortion coefficients |
| `calibration/camera_extrinsic/` | Homography for ground projection |
| `calibration/kinematics/` | Wheel radius, baseline, trim |
| `line_detector/line_detector_node/` | HSV thresholds for white/yellow/red line detection |
| `lane_control/lane_controller_node/` | PID gains (`k_d`, `k_theta`) and target speed |
| `keyboard_mapper/keyboard_mapper_node/` | Key-to-command mapping |

All parameters can also be tuned live using `dynamic_reconfigure`:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```
