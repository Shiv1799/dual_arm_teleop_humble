# Dual Arm Teleoperation with Hand Gestures
 
### Real-Time Two-Hand Gesture Control of Dual Franka Emika Panda Arms using ROS 2, ros2_control & MediaPipe
 
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue?logo=ros)](https://docs.ros.org/en/humble/)
[![MoveIt 2](https://img.shields.io/badge/MoveIt%202-Config-orange)](https://moveit.picknik.ai/humble/)
[![MediaPipe](https://img.shields.io/badge/MediaPipe-Hands-green?logo=google)](https://mediapipe.dev/)
[![Python](https://img.shields.io/badge/Python-3.10-yellow?logo=python)](https://www.python.org/)
[![Docker](https://img.shields.io/badge/Docker-Ready-2496ED?logo=docker)](https://www.docker.com/)
[![License](https://img.shields.io/badge/License-MIT-brightgreen)](LICENSE)
 
---
 
> Control TWO 7-DOF robotic arms simultaneously with your left and right hands. No gloves, no depth camera, no special hardware — just a standard 2D webcam, MediaPipe two-hand tracking, and computer vision.
 
---
 
## Demo
 
![Demo](docs/demo.gif)
 
<!-- Replace with your actual GIF after recording -->
 
| Left Hand — Left Arm | Right Hand — Right Arm | Pinch — Gripper | Fist — E-Stop |
|:---------------------:|:----------------------:|:---------------:|:-------------:|
| Move left hand to steer left arm | Move right hand to steer right arm | Pinch thumb + index to close | Close fist to freeze |
 
```text
+-------------------+     Per-Hand ROS 2 Topics     +-------------------+     JointTrajectory      +-------------------+
|                   |  /left/gesture/target_pose     |                   |  /left_arm_controller/   |                   |
|  Webcam + 2-Hand  |  /left/gesture/gripper_cmd     |  Dual Gesture     |    joint_trajectory      |  Dual Franka      |
|  Tracking Node    |  /right/gesture/target_pose    |  Bridge           |  /right_arm_controller/  |  Panda Arms       |
|  (MediaPipe)      |  /right/gesture/gripper_cmd    |  (Joint Mapping)  |    joint_trajectory      |  (14-DOF total)   |
|                   |  /{side}/gesture/hand_rotation  |                   |  /joint_states (fingers) |                   |
+-------------------+                                +-------------------+                          +-------------------+
     MediaPipe                                          Direct Joint                                   Real-Time
     21 Landmarks x2                                    Position Control                               Joint Trajectories
```
 
---
 
## Key Features
 
- **Simultaneous dual-arm control** — left hand drives the left Panda, right hand drives the right Panda, independently and in real time
- **End-effector rotation** — tilt your hand to rotate the robot wrist (joint 7), mapped from wrist-to-MCP angle
- **Pinch-to-grip with hysteresis** — close threshold at 0.07, open threshold at 0.09 prevents flickering
- **Fist emergency stop** — all five fingertips near palm triggers immediate halt
- **3D hand visualization in RViz** — 21-point hand skeleton rendered as MarkerArray in the robot workspace
- **Camera feed in RViz** — annotated webcam stream published as `sensor_msgs/Image` on `/gesture/camera_feed`
- **Single launch file** — one command brings up ros2_control, RViz, gesture tracking, and the bridge
- **No MoveIt Servo needed** — direct JointTrajectory publishing for minimal latency
- **Fully containerized** — Docker support for reproducible builds
 
---
 
## Architecture
 
```text
┌──────────────────────────────────────────────────────────────────────────────┐
│                          System Architecture                                 │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   ┌──────────────┐                                                           │
│   │   Webcam      │                                                          │
│   │  /dev/video0  │                                                          │
│   └──────┬───────┘                                                           │
│          │ BGR frames @ 30fps                                                │
│          ▼                                                                   │
│   ┌──────────────────────────────────────────────┐                           │
│   │  dual_gesture_tracker                        │                           │
│   │  ┌────────────────────────────────────────┐  │                           │
│   │  │ MediaPipe Hands (2 hands, 21 pts each) │  │                           │
│   │  └────────────┬───────────────────────────┘  │                           │
│   │               │                              │     ┌──────────────────┐  │
│   │  ┌────────────▼───────────────────────────┐  │     │ /gesture/        │  │
│   │  │ Per-Hand Gesture Interpretation        │  │     │  camera_feed     │  │
│   │  │  • Wrist XY → Robot workspace position │──┼────▶│  hand_markers    │  │
│   │  │  • Wrist-MCP angle → EE rotation       │  │     │  target_markers  │  │
│   │  │  • Pinch detect → Gripper (hysteresis) │  │     │  (RViz viz)      │  │
│   │  │  • Fist detect  → Emergency stop       │  │     └──────────────────┘  │
│   │  │  • EMA smoothing (alpha = 0.4)         │  │                           │
│   │  └────────────────────────────────────────┘  │                           │
│   └──────┬──────────────────┬────────────────────┘                           │
│          │                  │                                                │
│   /left/gesture/*    /right/gesture/*                                        │
│          │                  │                                                │
│          ▼                  ▼                                                │
│   ┌──────────────────────────────────────────────┐                           │
│   │  dual_gesture_bridge                         │                           │
│   │  ┌─────────────────────────────────────────┐ │                           │
│   │  │ ArmController (left)  ArmController (R) │ │                           │
│   │  │                                         │ │                           │
│   │  │ Gesture → Joint mapping:                │ │                           │
│   │  │  Hand X  → Joint 1 (shoulder rotation)  │ │                           │
│   │  │  Hand Y  → Joint 2 (lift) + J4 (elbow)  │ │                           │
│   │  │  Tilt    → Joint 7 (wrist rotation)     │ │                           │
│   │  │  Pinch   → Finger joints via /joint_states│ │                         │
│   │  └─────────────┬──────────┬────────────────┘ │                           │
│   │                │          │                   │                           │
│   │           /clock (100Hz)  │                   │                           │
│   └────────────────┼──────────┼───────────────────┘                          │
│                    │          │                                               │
│         JointTrajectory    JointState                                        │
│         (7 joints x2)     (finger joints)                                    │
│                    │          │                                               │
│                    ▼          ▼                                               │
│   ┌──────────────────────────────────────────────┐                           │
│   │  ros2_control (mock hardware)                │                           │
│   │  • left_arm_controller  (JointTrajectory)    │                           │
│   │  • right_arm_controller (JointTrajectory)    │                           │
│   │  • joint_state_broadcaster                   │                           │
│   └──────────────────────────────────────────────┘                           │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘
```
 
---
 
## Coordinate Mapping
 
Each hand is mapped to its respective arm's workspace region:
 
| Camera | Left Arm | Right Arm | Description |
|:------:|:--------:|:---------:|:------------|
| X (left/right) | -1.9 → -1.1 m | 1.1 → 1.9 m | Shoulder rotation (joint 1) |
| Y (up/down) | 0.1 → 0.7 m | 0.1 → 0.7 m | Lift (joint 2) + elbow (joint 4) |
| Hand tilt | +/- 1.5 rad | +/- 1.5 rad | Wrist rotation (joint 7) |
| — | Y = 0.4 m fixed | Y = 0.4 m fixed | No depth from 2D camera |
 
```text
Camera View (mirrored)                    World Frame (top-down)
┌─────────────────────┐
│  LEFT    │   RIGHT   │                  Left Panda        Right Panda
│  HAND    │   HAND    │                  (-1.5, 0)         (+1.5, 0)
│          │           │       ──▶            │                  │
│  ↕ Y     │   ↕ Y     │     mapping     ┌───┴───┐          ┌───┴───┐
│  ↔ X     │   ↔ X     │                 │  arm  │          │  arm  │
└─────────────────────┘                  └───────┘          └───────┘
```
 
---
 
## Gesture Reference
 
| Gesture | Detection Method | Threshold | Robot Action |
|:--------|:-----------------|:---------:|:-------------|
| **Open Hand** | Default state | — | Arm tracks hand position |
| **Hand Tilt** | atan2(wrist → middle MCP) | continuous | EE wrist rotation (joint 7) |
| **Pinch** | Thumb tip ↔ Index tip distance | < 0.07 | Close gripper (0.0 m) |
| **Release** | Thumb tip ↔ Index tip distance | > 0.09 | Open gripper (0.04 m) |
| **Fist** | All 5 fingertips within palm radius | < 0.10 | **Emergency stop** — freeze arm |
 
> Pinch uses **hysteresis** (close at 0.07, open at 0.09) to prevent rapid flickering between states.
 
---
 
## Tech Stack
 
| Component | Technology | Role |
|:----------|:-----------|:-----|
| Robot Simulation | **Dual Franka Emika Panda** (14-DOF) | Two 7-DOF arms with grippers |
| Middleware | **ROS 2 Humble** | Communication, TF2, launch system |
| MoveIt Config | **dual_arm_panda_moveit_config** (submodule) | URDF, SRDF, controller config |
| Control | **ros2_control** (mock hardware) | Joint trajectory execution |
| Hand Tracking | **MediaPipe Hands** (2-hand mode) | 21 2D landmarks per hand, ~30 fps |
| Computer Vision | **OpenCV** + **cv_bridge** | Frame capture, annotation, ROS Image publishing |
| Visualization | **RViz2** | 3D robot + hand markers + camera feed |
| Containerization | **Docker** | Reproducible environment |
 
---
 
## Package Structure
 
```text
dual_arm_teleop_humble/
├── src/
│   ├── dual_panda_gesture_bringup/        # Main dual-arm package
│   │   ├── dual_panda_gesture_bringup/
│   │   │   ├── dual_gesture_tracker.py    # MediaPipe 2-hand tracking + RViz viz
│   │   │   └── dual_gesture_bridge.py     # Gesture-to-joint mapping (both arms)
│   │   ├── launch/
│   │   │   ├── dual_panda_all.launch.py         # Full system launch (recommended)
│   │   │   └── dual_gesture_control.launch.py   # Gesture nodes only
│   │   ├── config/
│   │   │   ├── dual_workspace_bounds.yaml       # Workspace mapping + thresholds
│   │   │   ├── gripper_controllers.yaml         # Gripper controller definitions
│   │   │   └── gesture_control.rviz             # RViz layout
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   └── moveit_resources/                  # [submodule] MoveIt resources
│       └── dual_arm_panda_moveit_config/  # URDF, SRDF, controllers for dual arm
│
├── Dockerfile
├── run.sh
├── README.md
└── docs/
    └── demo.gif
```
 
---
 
## Quick Start
 
### Prerequisites
 
- Linux (Ubuntu 22.04 recommended)
- Docker (or native ROS 2 Humble)
- USB webcam
 
### 1. Clone with Submodules
 
```bash
git clone --recurse-submodules https://github.com/Shiv1799/dual_arm_teleop_humble.git
cd dual_arm_teleop_humble
```
 
If you already cloned without `--recurse-submodules`:
 
```bash
git submodule update --init --recursive
```
 
### 2. Docker (Recommended)
 
```bash
docker build -t dual-arm-gesture:latest .
./run.sh
```
 
### 3. Build Workspace (inside container or native)
 
```bash
source /opt/ros/humble/setup.bash
pip install mediapipe opencv-python
colcon build --symlink-install
source install/setup.bash
```
 
### 4. Run Everything
 
```bash
ros2 launch dual_panda_gesture_bringup dual_panda_all.launch.py
```
 
That's it. RViz opens with two Panda arms. After ~6 seconds the gesture tracker starts. Show both hands to the webcam and watch each arm follow its corresponding hand.
 
### Launch Arguments
 
```bash
ros2 launch dual_panda_gesture_bringup dual_panda_all.launch.py \
  camera_id:=0 \
  smoothing_factor:=0.4 \
  velocity_gain:=1.5
```
 
| Argument | Default | Description |
|:---------|:-------:|:------------|
| `camera_id` | `0` | Webcam device index |
| `smoothing_factor` | `0.4` | Hand position smoothing (higher = smoother) |
| `velocity_gain` | `1.5` | Arm responsiveness (higher = faster) |
 
---
 
## ROS 2 Topics
 
| Topic | Type | Hz | Description |
|:------|:-----|:--:|:------------|
| `/{side}/gesture/target_pose` | `geometry_msgs/Pose` | 30 | Target EE position from hand |
| `/{side}/gesture/gripper_command` | `std_msgs/Bool` | 30 | `True` = close, `False` = open |
| `/{side}/gesture/hand_rotation` | `std_msgs/Float64` | 30 | Hand tilt angle (radians) |
| `/{side}/gesture/emergency_stop` | `std_msgs/Bool` | 30 | `True` when fist detected |
| `/gesture/hand_markers` | `visualization_msgs/MarkerArray` | 30 | 3D hand skeleton in RViz |
| `/gesture/target_markers` | `visualization_msgs/MarkerArray` | 30 | Target position spheres |
| `/gesture/camera_feed` | `sensor_msgs/Image` | 30 | Annotated webcam frame |
| `/{side}_arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 5 | 7-joint arm commands |
| `/joint_states` | `sensor_msgs/JointState` | 10 | Finger positions (gripper) |
| `/clock` | `rosgraph_msgs/Clock` | 100 | Simulated clock for ros2_control |
 
> `{side}` is `left` or `right`
 
---
 
## Configuration & Tuning
 
<details>
<summary><b>Gesture Detection Thresholds</b> — <code>dual_workspace_bounds.yaml</code></summary>
 
```yaml
/**:
  ros__parameters:
    pinch_threshold: 0.07    # Lower = harder to trigger pinch
    pinch_hysteresis: 0.02   # Gap between close/open thresholds
    fist_threshold: 0.10     # Lower = harder to trigger e-stop
    smoothing_factor: 0.4    # 0.0 = raw, 1.0 = max smoothing
```
</details>
 
<details>
<summary><b>Workspace Bounds</b> — <code>dual_workspace_bounds.yaml</code></summary>
 
```yaml
/**:
  ros__parameters:
    # Left arm (camera left half)
    left_ws_x_min: -1.9
    left_ws_x_max: -1.1
    left_ws_z_min: 0.1
    left_ws_z_max: 0.7
    left_ws_y_fixed: 0.4
    # Right arm (camera right half)
    right_ws_x_min: 1.1
    right_ws_x_max: 1.9
    right_ws_z_min: 0.1
    right_ws_z_max: 0.7
    right_ws_y_fixed: 0.4
```
</details>
 
<details>
<summary><b>Joint Mapping Ranges</b> — <code>dual_gesture_bridge.py</code></summary>
 
| Joint | Range from HOME | Gesture Input |
|:------|:---------------:|:-------------|
| Joint 1 (shoulder rotation) | +/- 1.2 rad | Hand X position |
| Joint 2 (shoulder lift) | +/- 0.6 rad | Hand Y position |
| Joint 4 (elbow bend) | +/- 0.8 rad | Hand Y position |
| Joint 7 (wrist rotation) | +/- 1.5 rad | Hand tilt angle |
 
HOME position: `[0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]`
</details>
 
---
 
## Design Decisions
 
| Decision | Rationale |
|:---------|:----------|
| **Direct JointTrajectory** over MoveIt Servo | Dual-arm setup doesn't need Servo's IK — direct joint mapping is simpler, faster, and avoids two Servo instances |
| **`/joint_states` for grippers** over separate controller | Gripper controllers fail to spawn with mock hardware; publishing finger positions on `/joint_states` works reliably with `robot_state_publisher` |
| **Pinch hysteresis** (0.07/0.09) | Prevents rapid open/close flickering when finger distance hovers near threshold |
| **Wrist-to-MCP angle** for EE rotation | Natural, intuitive mapping — tilt your hand and the robot wrist follows |
| **Per-hand handedness classification** | MediaPipe's built-in L/R label maps each hand to its corresponding arm automatically |
| **100 Hz `/clock` publishing** | ros2_control mock hardware needs advancing sim_time to execute trajectories |
| **6-second delayed gesture startup** | Ensures ros2_control and controllers are fully active before gesture commands arrive |
| **EMA smoothing** on hand position | Eliminates jitter from MediaPipe landmark noise without adding latency |
 
---
 
## Troubleshooting
 
<details>
<summary><b>Arms don't move</b></summary>
 
1. Check controllers: `ros2 control list_controllers` — expect `left_arm_controller [active]`, `right_arm_controller [active]`
2. Check trajectory topic: `ros2 topic echo /left_arm_controller/joint_trajectory`
3. Check clock is publishing: `ros2 topic hz /clock` — should be ~100 Hz
4. If you see `servo_node` or `gripper_cmd` errors, your build is **stale** — rebuild:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```
</details>
 
<details>
<summary><b>Gripper doesn't open/close in RViz</b></summary>
 
1. Check finger joint states: `ros2 topic echo /joint_states` — look for `left_panda_finger_joint1`
2. Check tracker detects pinch: look for `Pinch - GRIP` in the webcam overlay
3. Verify bridge logs: `LEFT Gripper: CLOSE` / `LEFT Gripper: OPEN`
</details>
 
<details>
<summary><b>Webcam not found</b></summary>
 
1. Check devices: `ls /dev/video*`
2. Try different camera_id: `camera_id:=1`
3. Docker: ensure `--device /dev/video0` is passed (check `run.sh`)
</details>
 
<details>
<summary><b>Only one hand detected</b></summary>
 
1. Ensure both hands are clearly visible in the frame
2. Good lighting improves MediaPipe detection
3. Hands must be sufficiently separated (not overlapping)
</details>
 
<details>
<summary><b>Arm moves erratically</b></summary>
 
1. Decrease `velocity_gain` (try `1.0`)
2. Increase `smoothing_factor` (try `0.6`)
3. Ensure good, consistent lighting
</details>
 
<details>
<summary><b>Camera feed not in RViz</b></summary>
 
1. In RViz: **Add** → **By topic** → `/gesture/camera_feed` → **Image**
2. Check topic: `ros2 topic hz /gesture/camera_feed` — should be ~30 Hz
</details>
 
---
 
## Known Limitations
 
- **2D tracking only** — no depth control (Y-axis fixed at 0.4 m) since a single webcam provides no depth information
- **No collision avoidance** — joint positions are computed via direct mapping, not motion planning; arms could collide if workspaces overlap
- **Gripper visualization only** — finger positions update the RViz model via `/joint_states` but don't use a dedicated ros2_control gripper controller
- **Lighting dependent** — MediaPipe hand detection requires reasonable, consistent lighting
- **Handedness swapping** — MediaPipe may occasionally swap left/right labels; keep hands well-separated
 
---
 
## Future Improvements
 
- [ ] Depth estimation using monocular depth models (MiDaS) for full 3D control
- [ ] Dedicated gripper controller integration with proper hardware interface
- [ ] Collision avoidance between the two arms using MoveIt planning scene
- [ ] Bimanual coordination gestures (e.g., two-hand grab for heavy objects)
- [ ] Integration with real Franka hardware via `franka_ros2`
- [ ] Voice command overlay for mode switching
- [ ] Recording and replay of dual-arm gesture trajectories
- [ ] Wrist orientation tracking for full 6-DOF EE control
 
---
 
## References
 
- [MediaPipe Hands — Multi-Hand Tracking](https://mediapipe.dev/solutions/hands)
- [MoveIt 2 — Motion Planning Framework](https://moveit.ros.org/)
- [ros2_control — Hardware Abstraction](https://control.ros.org/humble/)
- [Franka Emika Panda — Robot Model](https://www.franka.de/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
 
---
 
<p align="center">
  Built with ROS 2 Humble | ros2_control | MediaPipe | OpenCV
</p>
