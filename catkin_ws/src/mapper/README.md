# mapper (joy_mapper)

Maps joystick input (`sensor_msgs/Joy`) to `rosky_msgs/CarControl` commands and publishes them at a fixed rate using zero-order hold.

## Testing

1. Connect a joystick
2. `roslaunch mapper joy_mapper_test.launch`
3. Push joystick buttons — ROSKY should move

## Node: joy_mapper

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `~pub_timestep` | `0.02` (50 Hz) | Interval between `CarControl` publishes (seconds) |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `joy` | `sensor_msgs/Joy` | Joystick input from the `joy` package |

Left stick vertical axis → speed. Right stick horizontal axis → steering.

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~joy_control` | `rosky_msgs/CarControl` | Speed `[-1, 1]` and steering `[-1, 1]`; positive steering = left |
