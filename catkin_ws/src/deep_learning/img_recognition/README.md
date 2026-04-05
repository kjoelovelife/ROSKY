# img_recognition

AlexNet-based image classification package. Classifies camera frames into user-defined categories (e.g. `free` / `blocked`) and publishes confidence scores.

See the [deep_learning README](../README.md) for the full step-by-step workflow.

## Nodes

| Node | Script | Description |
|------|--------|-------------|
| `save_image` | `src/` | Saves labeled images via ROS service trigger |
| `train_model` | `src/train_model.py` | Trains AlexNet on collected images |
| `inference_model` | `src/inference_model.py` | Runs trained model on live camera feed |
| `inference_to_reaction` | `src/inference_to_reaction.py` | Converts inference output to motion commands |

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~inference` | `rosky_msgs/InferenceResult` | Label names and per-class confidence scores |

## Directory Layout

```
img_recognition/
├── image/          # Collected training images, one subfolder per label
├── model/          # Saved .pth model files + recording.yaml
├── param/          # train_model.yaml, inference_model.yaml, image_label.yaml
└── src/            # Node scripts
```
