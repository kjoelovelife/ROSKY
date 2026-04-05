# rosky_slam

Meta-package providing launch files and RViz configuration for SLAM (Simultaneous Localization and Mapping) using the YDLidar X4 and gmapping.

## Launch Files

| Launch File | Description |
|-------------|-------------|
| `gmapping.launch` | Starts `rf2o_laser_odometry` + `slam_gmapping` to build a 2D occupancy grid map |

```bash
roslaunch rosky_slam gmapping.launch
```

## Dependencies

- `rf2o_laser_odometry` — laser-based odometry (see `../rf2o_laser_odometry/`)
- `ydlidar` — YDLidar X4 driver (see `../ydlidar/`)
- `ros-melodic-slam-gmapping`
- `ros-melodic-map-server`

## RViz

A default RViz configuration for visualizing the map and laser scan is provided at `rviz/default.rviz`. It is copied to `~/.rviz/default.rviz` by `~/ROSKY/setup/set_package_param.sh`.
