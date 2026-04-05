# self_driving

Combines `img_recognition` (obstacle classification) and `road_following` (line regression) to enable fully autonomous driving: ROSKY follows the road and reacts when it detects a blocked path.

Requires trained models from both `img_recognition` and `road_following` before use.

See the [deep_learning README](../README.md) for the full workflow.

## Nodes

| Node | Script | Description |
|------|--------|-------------|
| `self_driving_inference_model` | `src/self_driving_inference_model.py` | Runs both models in parallel on live camera feed |
| `self_driving_inference_to_reaction` | `src/self_driving_inference_to_reaction.py` | Merges classification and regression outputs into motor commands |

## Customization

Edit `src/self_driving_inference_to_reaction.py` to define custom robot behaviors in response to specific recognized labels (e.g. stop on `blocked`, slow down on `caution`).

## Launch

```bash
roslaunch self_driving inference.launch reaction:=true
# Tune parameters live:
rosrun rqt_reconfigure rqt_reconfigure   # select "self_driving_inference_to_reaction"
```
