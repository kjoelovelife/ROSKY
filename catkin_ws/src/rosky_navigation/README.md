# rosky_navigation

Meta-package providing launch files and parameter configuration for autonomous navigation using the ROS Navigation Stack (move_base).

## Launch Files

| Launch File | Description |
|-------------|-------------|
| `navigation.launch` | Full navigation stack with move_base, costmaps, and local planner |
| `nav_control.launch` | Navigation controller only (without map server) |
| `rf2o.launch` | Laser odometry node (from `rf2o_laser_odometry`) |

```bash
# Start navigation (requires a map from rosky_slam first)
roslaunch rosky_navigation navigation.launch
```

## Dependencies

- `ros-melodic-navigation` — move_base, costmap2d, DWA local planner
- `ros-melodic-teb-local-planner` — TEB local planner (alternative)
- `rf2o_laser_odometry` — odometry source
- `ydlidar` — laser scan source

## Configuration

Navigation parameters (costmap resolution, inflation radius, planner settings) are in the `param/` directory. Adjust these to tune navigation behavior for the ROSKY's physical dimensions and motor characteristics.
