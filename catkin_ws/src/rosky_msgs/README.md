# rosky_msgs

Custom ROS message and service definitions shared across all ROSKY packages.

## Messages (45)

Key types used throughout the system:

| Message | Description |
|---------|-------------|
| `WheelsCmd` / `WheelsCmdStamped` | Individual wheel velocity commands |
| `CarControl` | High-level speed + steering angle command |
| `LanePose` | Lane-following state: lateral offset `d` and heading error `phi` |
| `Segment` / `SegmentList` | Detected line segments with color labels (white/yellow/red) |
| `FSMState` | Finite state machine mode (e.g. LANE_FOLLOWING, INTERSECTION_CONTROL) |
| `AprilTagDetection` / `AprilTagDetectionArray` | AprilTag marker detections |
| `ObstacleImageDetection` / `ObstacleProjectedDetection` | Obstacle detections in image and ground frame |
| `VehiclePose` / `Trajectory` | Robot pose and motion history |
| `Twist2DStamped` | 2D velocity (v, omega) with timestamp |
| `BoolStamped` | Stamped boolean, used for stop-line and coordination signals |
| `AntiInstagramTransform` | Color correction transform coefficients |
| `KinematicsParameters` / `KinematicsWeights` | Wheel kinematics calibration data |

## Services (2)

| Service | Description |
|---------|-------------|
| `SetFSMState` | Set the active FSM state |
| `SetValue` | Generic float value setter |

## Usage

Add `rosky_msgs` to your `package.xml` dependencies:

```xml
<depend>rosky_msgs</depend>
```

Import in Python:

```python
from rosky_msgs.msg import CarControl, LanePose, WheelsCmdStamped
from rosky_msgs.srv import SetFSMState
```
