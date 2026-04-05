# anti_instagram

Color correction node that normalizes camera images for varying lighting conditions using a KMeans-based approach. It computes a linear transform in HSV space and publishes the transform coefficients so that downstream nodes (e.g. `line_detector`) can apply consistent color thresholds regardless of ambient light.

## Node

**`anti_instagram_node`** (`src/anti_instagram_node.py`)

Subscribes to raw compressed images, runs the KMeans color correction algorithm, and publishes the transform.

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~uncorrected_image/compressed` | `sensor_msgs/CompressedImage` | Raw image from camera |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~corrected_image/compressed` | `sensor_msgs/CompressedImage` | Color-corrected image |
| `~transform` | `rosky_msgs/AntiInstagramTransform` | Color transform coefficients |
| `~health` | `rosky_msgs/AntiInstagramHealth` | Node health / correction quality |

## Core Algorithm

The algorithm lives in `include/anti_instagram/AntiInstagram.py`. It:
1. Samples pixels from the image
2. Clusters them using KMeans to identify dominant colors
3. Computes a linear mapping from the detected colors to canonical reference colors
4. Applies the mapping as a per-channel gain and offset in HSV space

## Tests

Unit and performance tests are in the `tests/` directory:

```bash
cd ~/ROSKY/catkin_ws
catkin_make run_tests --pkg anti_instagram
```
