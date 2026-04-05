# rosky_visualizer

Publishes RViz visualization markers for the ROSKY robot state, including lane pose, detected segments, and vehicle position.

## Nodes

| Node | Script | Description |
|------|--------|-------------|
| `rosky_visualizer_node` | `src/rosky_visualizer.py` | Publishes `visualization_msgs/MarkerArray` for RViz |
| `fw_rosky_visualizer_node` | `src/fw_rosky_visualizer.py` | Forward-wheel variant visualization |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~lane_pose` | `rosky_msgs/LanePose` | Current lane pose estimate |
| `~segment_list` | `rosky_msgs/SegmentList` | Detected line segments |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~markers` | `visualization_msgs/MarkerArray` | RViz markers for lane and robot state |

## Usage

The visualizer is typically started alongside the lane-following stack. Open RViz with the default configuration from `rosky_slam` to see the output:

```bash
rviz -d ~/.rviz/default.rviz
```
