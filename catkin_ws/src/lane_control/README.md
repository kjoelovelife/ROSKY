# lane_control

PID controller that converts the `LanePose` estimate from `lane_filter` into a `CarControl` command (speed + steering angle) sent to `ominibot_car`.

## Nodes

| Node | Script | Description |
|------|--------|-------------|
| `lane_controller_node` | `scripts/lane_controller_node.py` | Main PID controller |
| `lane_controller_reconfigure` | `scripts/lane_controller_reconfigure.py` | Dynamic reconfigure server for live gain tuning |
| `vicon_for_lane_node` | `scripts/vicon_for_lane_node.py` | Optional Vicon motion-capture integration |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~lane_pose` | `rosky_msgs/LanePose` | Estimated `d` (lateral offset) and `phi` (heading error) from `lane_filter` |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~car_control` | `rosky_msgs/CarControl` | Speed and steering angle command |

## Control Law

The steering command is computed as:

```
omega = k_d * d + k_theta * phi * steer_gain
```

Where `k_d` and `k_theta` are PID gains and `steer_gain` is an overall steering scale. Forward speed `v` is set to a constant `v_bar`.

## Configuration

Gains are stored in `rosky_base/config/baseline/lane_control/lane_controller_node/<hostname>.yaml` and can be tuned live:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```
