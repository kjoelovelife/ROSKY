# deep_learning

Meta-package containing three deep learning packages for autonomous driving on ROSKY. Based on [NVIDIA JetBot](https://github.com/NVIDIA-AI-IOT/jetbot).

Each sub-package follows the same three-step workflow: **data collection → training → inference**.

| Package | Model | Task |
|---------|-------|------|
| [`img_recognition`](img_recognition/) | AlexNet (classification) | Recognize objects (e.g. free/blocked) |
| [`road_following`](road_following/) | ResNet18 (regression) | Follow a colored line |
| [`self_driving`](self_driving/) | Both | Drive the road while recognizing obstacles |

Complete `img_recognition` and `road_following` before using `self_driving`.

---

## img_recognition

Classifies camera images into user-defined categories (e.g. `free` / `blocked`).

### Step 1 — Data Collection

Create a label folder and start saving images:

```bash
rosrun img_recognition mkdir.py -n free
rosrun img_recognition mkdir.py -n blocked

roslaunch img_recognition save_image.launch label:=free
# In a second terminal, trigger image capture:
rosservice call /HOSTNAME/save_image/save_image_action -- true
# Stop capture:
rosservice call /HOSTNAME/save_image/save_image_action -- false
```

Images are saved to `img_recognition/image/<label>/`. Repeat for each label. Vary object angles and backgrounds to avoid overfitting.

### Step 2 — Training

Edit `img_recognition/param/train_model.yaml`, then:

```bash
roslaunch img_recognition train_model.launch
```

The trained model is saved to `img_recognition/model/`. Training is complete when you see `"Now you can press [ctrl] + [c]"` — the red error text at the end is normal (the node exits intentionally).

### Step 3 — Inference

Set `model_pth` in `img_recognition/param/inference_model.yaml` to your trained model name, then:

```bash
roslaunch img_recognition inference.launch
# Monitor output:
rostopic echo /HOSTNAME/inference_model/inference
```

---

## road_following

Uses regression to predict a target (x, y) point on the road ahead, enabling ROSKY to follow a colored line.

### Step 1 — Data Collection

```bash
roslaunch road_following display_xy.launch
# On another machine, view the annotated image:
rosrun rqt_image_view rqt_image_view image:=/HOSTNAME/display_xy/image/raw/draw_xy_line
# Adjust the x/y target and capture images:
rosrun rqt_reconfigure rqt_reconfigure   # select "display_xy"
```

Use `rqt_reconfigure` to set `X`, `Y`, and enable `save_image` to capture frames.

### Step 2 — Training

Edit `road_following/param/train_model.yaml`, then:

```bash
roslaunch road_following train_model.launch
```

### Step 3 — Inference

Set `model_pth` in `road_following/param/inference_model.yaml`, then:

```bash
roslaunch road_following inference.launch reaction:=true
# Tune PID live:
rosrun rqt_reconfigure rqt_reconfigure   # select "road_inference_to_reaction"
```

Key `rqt_reconfigure` parameters:

| Parameter | Effect |
|-----------|--------|
| `speed_gain` | Forward speed |
| `steering_gain` / `steering_kd` | Reduce if wobbling |
| `steering_bias` | Correct left/right drift |
| `save_parameter` | Save current PID to `param/<hostname>_pid.yaml` |

---

## self_driving

Combines `img_recognition` and `road_following` for fully autonomous driving — following the road while reacting to recognized obstacles.

Requires trained models from both sub-packages first.

```bash
roslaunch self_driving inference.launch reaction:=true
# Tune live:
rosrun rqt_reconfigure rqt_reconfigure   # select "self_driving_inference_to_reaction"
```

To customize the robot's reaction behavior, edit `self_driving/src/self_driving_inference_to_reaction.py`.
