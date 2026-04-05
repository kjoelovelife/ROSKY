# jetson_camera

ROS driver for the CSI fisheye camera on the Jetson Nano. Captures frames from the camera, applies optional preprocessing (undistortion, decoding), and publishes compressed images for the rest of the perception pipeline.

## Nodes

| Node | Script | Description |
|------|--------|-------------|
| `jetson_camera_node` | `src/jetson_camera.py` | Main camera driver; reads from CSI via GStreamer and publishes images |
| `decoder_node` | `scripts/decoder_node.py` | Decodes compressed images to raw `sensor_msgs/Image` |
| `img_process_node` | `scripts/img_process_node.py` | Optional image preprocessing (resize, flip) |
| `cam_info_reader_node` | `scripts/cam_info_reader_node.py` | Reads and publishes `CameraInfo` from a YAML file |

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~image/compressed` | `sensor_msgs/CompressedImage` | Compressed camera frames |
| `~camera_info` | `sensor_msgs/CameraInfo` | Intrinsic calibration |

## Parameters

Camera info YAML files are stored in `camera_info/`. Per-robot files follow the naming convention `<hostname>.yaml` and are set up by `~/ROSKY/setup/set_package_param.sh`.

## Calibration

Run intrinsic calibration with a checkerboard target:

```bash
roslaunch rosky_base intrinsic_calibration.launch veh:=rosky01
```

The resulting YAML is saved to `config/baseline/calibration/camera_intrinsic/<hostname>.yaml` in `rosky_base`.
