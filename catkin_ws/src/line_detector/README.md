# line_detector

Detects colored lane markings (white, yellow, red) in camera images using HSV thresholding and edge detection. Outputs a list of line segments that the `ground_projection` and `lane_filter` packages use to estimate lane position.

## Nodes

| Node | Script | Description |
|------|--------|-------------|
| `line_detector_node` | `src/line_detector_node.py` | Primary detector; publishes `SegmentList` |
| `line_detector_node2` | `src/line_detector_node2.py` | Alternative implementation |
| `line_detector_node_gc` | `src/line_detector_node_gc.py` | Version with ground-crop preprocessing |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~corrected_image/compressed` | `sensor_msgs/CompressedImage` | Color-corrected image (from `anti_instagram`) |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~segment_list` | `rosky_msgs/SegmentList` | Detected line segments with color labels |
| `~image_with_lines` | `sensor_msgs/Image` | Debug visualization with overlaid detections |

## Configuration

HSV threshold parameters are in `rosky_base/config/baseline/line_detector/line_detector_node/<hostname>.yaml`. They define the hue/saturation/value ranges for each color class and can be tuned live:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```
