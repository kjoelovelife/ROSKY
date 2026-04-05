# road_following

ResNet18-based regression package that predicts a target (x, y) point on the road ahead, allowing ROSKY to follow a colored line.

See the [deep_learning README](../README.md) for the full step-by-step workflow.

## Nodes

| Node | Script | Description |
|------|--------|-------------|
| `display_xy` | `src/` | Shows live camera feed with annotated (x, y) target point |
| `train_model` | `src/train_model.py` | Trains ResNet18 on collected (image, x, y) pairs |
| `road_model_inference` | `src/road_model_inference.py` | Runs trained model; publishes predicted steering angle |
| `road_inference_to_reaction` | `src/road_inference_to_reaction.py` | PID controller converting angle prediction to motor commands |

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~inference` | custom | Predicted `angle` and `angle_last` values |

## Directory Layout

```
road_following/
├── image/dataset_xy/   # Collected training images with (x, y) labels encoded in filenames
├── model/              # Saved .pth model files + recording.yaml
├── param/              # train_model.yaml, inference_model.yaml, <hostname>_pid.yaml
└── src/                # Node scripts
```
