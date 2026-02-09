# ROS Demo - Beverage Detection Pipeline

Learning ROS2 by building a perception pipeline for retail robotics's shelf-stocking robots.

**Status: Working!** Full perception pipeline with Gazebo simulation. Robot arm with camera, ros2_control, BoT-SORT tracking.

## Quick Start (Linux PC with NVIDIA GPU)

```bash
# 1. Clone and setup
git clone <repo>
cd ros-perception-demo
cp .env.example .env
# Edit .env for your paths

# 2. Build and run
docker compose build
docker compose up -d
docker exec -it ros_demo bash

# 3. Inside container: build ROS2 workspace
cd /ros_ws
colcon build
source install/setup.bash

# 4. Gazebo Simulation (robot arm + shelf) - RECOMMENDED
# Terminal 1: Launch Gazebo (spawns robot, shelf, starts ros2_control)
ros2 launch robot_description simulation.launch.py
# Terminal 2: Run tracker (subscribes to /camera/image_raw from Gazebo)
ros2 run perception_demo tracker_node
# Terminal 3: Visualize detections
ros2 run rqt_image_view rqt_image_view
# → Select: /camera/image_raw (arm camera) or /tracks/image (with boxes + trails)
# Terminal 4: Move the arm to look at shelf
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_joint], points: [{positions: [0.0, 1.0, -1.5, 0.0], time_from_start: {sec: 2}}]}}"

# 5. Alternative: Real Camera (instead of Gazebo)
# Terminal 1: Camera source (pick one)
ros2 run perception_demo camera_node --ros-args -p source:=webcam  # USB webcam
ros2 run perception_demo camera_node --ros-args -p source:=sim     # Static test images
# Terminal 2: Perception (pick one)
ros2 run perception_demo tracker_node --ros-args -p tracker:=botsort   # Detection + tracking (BoT-SORT, default)
ros2 run perception_demo tracker_node --ros-args -p tracker:=bytetrack # Detection + tracking (ByteTrack)
ros2 run perception_demo detector_node --ros-args -p model:=yolo       # Detection only (YOLO)
# Terminal 3: Depth estimation (optional)
ros2 run perception_demo depth_node --ros-args -p method:=midas    # Monocular depth
# Terminal 4: Visualize
ros2 run rqt_image_view rqt_image_view
# → Topics: /tracks/image, /detections/image, /depth/image
```

## Architecture

### ROS2 Node Pipeline

```
┌─────────────────┐     /camera/image_raw   ┌──────────────────┐     /detections
│   camera_node   │ ───────────────────────▶│  detector_node   │ ──────────────────▶
│                 │    sensor_msgs/Image    │                  │   Detection2DArray
└─────────────────┘                         └──────────────────┘
        │                                           │
        │ source:=                                  │ model:=
        │ • sim (static images) ✓                   │ • yolo (YOLOv8) ✓
        │ • webcam (USB camera) ✓                   │
        │ • gazebo (ros2 launch) ✓           ┌──────────────────┐     /tracks
        │                                    │  tracker_node    │ ──────────────────▶
        │                                    │ (with trail viz) │   Detection2DArray
        │                                    └──────────────────┘   + track IDs
        │                                           │ tracker:=
        │                                           │ • botsort (default) ✓
        │                                           │ • bytetrack ✓
        │                                           │
        │                                    ┌──────────────────┐
        │                                    │   depth_node     │ → /depth/image
        │                                    └──────────────────┘
        │                                           │ method:=
        │                                           │ • midas (monocular) ✓
        │                                           │ • realsense (future)
        │
        │                                    Future nodes:
        │                                    • pose_node (6DoF pose estimation)
        │                                    • segmentation_node
        │
                                                    │ /tracks/image
                                                    ▼
                                            (annotated image with trajectory trails)
```

**YOLOv8**: Auto-downloads on first run (~6MB), runs on PyTorch + CUDA.

### Full Pipeline Vision (retail robotics-style)

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                              PERCEPTION                                       │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  camera_node ──▶ detector_node ──▶ depth_node ──▶ tracking_node             │
│       │              │                 │               │                     │
│   RGB image     2D bboxes          depth map      object IDs                │
│                      │                 │               │                     │
│                      └────────┬────────┘               │                     │
│                               ▼                        │                     │
│                        pose_node ◀─────────────────────┘                     │
│                            │                                                 │
│                      6DoF object pose                                        │
│                            │                                                 │
├────────────────────────────┼─────────────────────────────────────────────────┤
│                            ▼                                                 │
│                     grasp_node                                               │
│                          │                                                   │
│                   grasp points                                               │
│                          │                                                   │
├──────────────────────────┼───────────────────────────────────────────────────┤
│                          ▼                          PLANNING & CONTROL       │
│                    planner_node ──▶ controller_node ──▶ robot                │
│                          │               │                                   │
│                    motion plan      joint commands                           │
│                                                                              │
└──────────────────────────────────────────────────────────────────────────────┘

Current:  ✅ camera_node, detector_node, depth_node, tracker_node, Gazebo simulation
Future:   ⬜ pose_node, segmentation_node, grasp_node
Later:    ⬜ planner_node, controller_node

Gazebo Simulation:
  ✅ v1: Simple 4-DOF arm + shelf + simulated camera + ros2_control
  ⬜ v2: Custom TX-style SCARA arm + convenience store shelf with real product models
```

### Development Workflow

```
┌─────────────────┐      git push      ┌─────────────────┐
│   MacBook Pro   │ ─────────────────▶ │     Gitea       │
│   (code editor) │                    │   (git server)  │
└─────────────────┘                    └─────────────────┘
                                               │
                                               │ git pull
                                               ▼
                                       ┌─────────────────────────────────────┐
                                       │      Linux PC (Pop!_OS + RTX)       │
                                       │  ┌───────────────────────────────┐  │
                                       │  │   Docker Container            │  │
                                       │  │  ┌─────────────────────────┐  │  │
                                       │  │  │ ROS2 + PyTorch + CUDA   │  │  │
                                       │  │  │ camera_node             │  │  │
                                       │  │  │ perception_node         │  │  │
                                       │  │  └─────────────────────────┘  │  │
                                       │  └───────────────────────────────┘  │
                                       │           │                         │
                                       │           │ X11 (GUI)               │
                                       │           ▼                         │
                                       │      ┌─────────┐                    │
                                       │      │ Monitor │ ← Gazebo/RViz      │
                                       │      └─────────┘                    │
                                       └─────────────────────────────────────┘
                                               │
                                               │ Future: same code runs on
                                               ▼
                                       ┌─────────────────┐
                                       │  Robot (Jetson) │
                                       │  Real cameras   │
                                       └─────────────────┘
```

**The magic:** Same ROS2 code runs in simulation (Docker) and on real robot. Just swap camera source.

### Visualization

View detections in real-time:

```bash
# Inside container, Terminal 3:
ros2 run rqt_image_view rqt_image_view
# Select topic: /detections/image
```

Or echo detection messages:
```bash
ros2 topic echo /detections
```

---

## ROS Core Concepts

### What is ROS?
- **Robot Operating System** - not actually an OS, but a middleware/framework
- Provides communication between processes (nodes)
- Industry standard for robotics software
- Think of it like Flask/Django for web → ROS for robots

### Key Concepts

| Concept | Description | Example |
|---------|-------------|---------|
| **Node** | A separate OS process that does one thing well | `detection_node`, `camera_node` |
| **Topic** | Named channel for messages (pub/sub) | `/camera/image`, `/detections` |
| **Message** | Data structure sent over topics | `sensor_msgs/Image`, `geometry_msgs/Pose` |
| **Publisher** | Node that sends messages to a topic | Camera publishes images |
| **Subscriber** | Node that receives messages from a topic | Detector subscribes to images |
| **Service** | Request/response (synchronous) | "Take a photo" → returns image |
| **Action** | Long-running task with feedback | "Navigate to shelf" → progress updates |

### ROS1 vs ROS2

| | ROS1 | ROS2 |
|---|---|---|
| Python API | `rospy` | `rclpy` |
| C++ API | `roscpp` | `rclcpp` |
| Master | Required (`roscore`) | No master (DDS) |
| Real-time | Limited | Better support |
| Status | Legacy (ends 2025) | Current standard |

We'll use **ROS2 Humble** (LTS, supported until 2027).

### Nodes in Detail - Are They Really Separate Processes?

**Yes! Each node is a separate OS process.**

```bash
# When you run nodes, you can see them as separate processes:
$ ps aux | grep ros
user  1234  python3 detection_node    # Process 1
user  1235  python3 camera_node       # Process 2
user  1236  arm_controller_node       # Process 3 (could be C++)
```

**In production robots:**
- Nodes can be **Python** (`rclpy`) or **C++** (`rclcpp`)
- C++ often used for performance-critical nodes (control loops, real-time)
- Python often used for perception, high-level logic
- Nodes can run on **same machine** or **distributed across multiple computers**
- Communication via **DDS** (Data Distribution Service) - handles networking automatically

**Example: Real robot setup**
```
┌─────────────────────┐     Network      ┌─────────────────────┐
│  Jetson (on robot)  │◄────────────────►│  Workstation (GPU)  │
│  - motor_control    │                  │  - perception_node  │
│  - sensor_drivers   │                  │  - planning_node    │
│  - arm_controller   │                  │  - visualization    │
└─────────────────────┘                  └─────────────────────┘
```

DDS handles message passing across the network transparently.

**Why separate processes?**
- **Isolation** - one node crashes, others keep running
- **Modularity** - swap out perception node without touching arm control
- **Distributed** - spread across multiple computers
- **Language flexibility** - mix Python and C++ nodes

### Common Message Types

```
sensor_msgs/Image          # Camera images (RGB, depth)
sensor_msgs/PointCloud2    # 3D point clouds (LiDAR, depth camera)
geometry_msgs/Pose         # Position (x,y,z) + orientation (quaternion)
geometry_msgs/Twist        # Velocity (linear + angular)
std_msgs/String            # Simple string
std_msgs/Float32           # Simple float
```

### TF (Transforms)

Coordinate frame transforms - essential for perception:
- Where is the object relative to the camera?
- Where is the camera relative to the robot base?
- Where is the robot relative to the world?

```
world
  └── robot_base
        └── arm_link_1
              └── arm_link_2
                    └── camera_link
```

`tf2` library lets you query: "Where is object X in the robot_base frame?"

---

## If You've Built a Distributed System Before, You Already Know ROS

**Example: Quant trading system architecture = ROS architecture**

| Quant Trading System | ROS Equivalent |
|----------------------|----------------|
| Separate processes (Python/Golang/Rust/C++) | Nodes |
| Redis pub/sub | Topics (via DDS) |
| Websocket connector process | Camera driver node |
| Strategy/live trading process | Planning/control node |
| Data processing process | Perception node |
| Message queues | Topics with message types |

If you've built microservices or distributed systems, you already understand ROS conceptually.

### Redis Pub/Sub vs DDS (ROS2)

| Aspect | Redis Pub/Sub | DDS (ROS2 Topics) |
|--------|---------------|-------------------|
| Architecture | Central broker (Redis server) | Peer-to-peer (no broker) |
| Discovery | Manual (connect to Redis) | Auto-discovery on network |
| QoS (Quality of Service) | Basic | Rich (reliability, deadline, liveliness) |
| Real-time | No guarantees | Designed for real-time |
| Message types | Strings/bytes (you serialize) | Strongly typed (defined .msg files) |
| Use case | General purpose | Robotics, defense, aerospace |
| Failure mode | Redis dies = all dead | No single point of failure |

**Key DDS features for robotics:**
- **Reliability**: "Deliver this message even if network hiccups" vs "best effort"
- **Deadline**: "Warn me if message doesn't arrive within 100ms"
- **Liveliness**: "Detect if a node crashed"
- **History**: "Keep last N messages for late subscribers"

**In practice:** DDS is more robust for safety-critical robotics. Redis is fine for trading/web apps where occasional message loss is acceptable.

### How Perception Fits in ROS

```
Camera Driver Node                    Your Perception Node                 Planning Node
     │                                       │                                  │
     │ publishes                             │ subscribes                       │ subscribes
     ▼                                       ▼                                  ▼
/camera/image_raw ──────────────────▶ runs YOLO ──────────────────▶ /detections
                                             │
                                             │ publishes
                                             ▼
                                      DetectionArray
                                      - class: "coca_cola"
                                      - bbox: [x, y, w, h]
                                      - confidence: 0.95
                                      - position_3d: [x, y, z]
```

## Project Structure

```
ros-perception-demo/
├── README.md
├── docker-compose.yml        # Container config (GPU, volumes, X11)
├── .env.example              # Template for local paths
├── docker/
│   └── Dockerfile            # ROS2 Humble + PyTorch + CUDA + ros2_control
├── src/
│   ├── perception_demo/      # ROS2 Python package (perception nodes)
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── perception_demo/
│   │       └── nodes/
│   │           ├── camera/   # Camera sources
│   │           │   ├── sim.py      # Static test images
│   │           │   └── webcam.py   # USB webcam
│   │           ├── depth/    # Depth estimation
│   │           │   └── midas.py    # Monocular (MiDaS)
│   │           ├── detector/ # Object detection
│   │           │   └── yolo.py     # YOLOv8
│   │           └── tracker/  # Object tracking
│   │               └── yolo_tracker.py  # BoT-SORT / ByteTrack
│   └── robot_description/    # ROS2 CMake package (Gazebo simulation)
│       ├── urdf/
│       │   └── robot.urdf.xacro    # 4-DOF arm with camera
│       ├── worlds/
│       │   └── shelf.world         # Convenience store shelf
│       ├── config/
│       │   └── controllers.yaml    # ros2_control config
│       └── launch/
│           └── simulation.launch.py
├── data/
│   └── test_images/          # Test images
└── models/                   # Model weights (*.pt gitignored)
```

### Why is everything under robot_description?

**ROS convention.** For simple projects, one package holds everything about the robot:
- URDF/meshes (robot model)
- Worlds (environments for this robot)
- Controllers (ros2_control config)
- Launch files

**For larger projects, you'd split:**
```
my_robot_description/   # Just URDF, meshes
my_robot_gazebo/        # Worlds, Gazebo plugins
my_robot_control/       # Controllers
my_robot_bringup/       # Launch files
```

## Setup

### Prerequisites
- Linux PC with NVIDIA GPU (tested: Pop!_OS 22.04, RTX 2060)
- Docker with NVIDIA Container Toolkit
- Git

### Environment Variables (.env)

```bash
# External data path for large files (not in git)
ROS_DEMO_DATA=/mnt/your-drive/ros-perception-demo-data

# X11 display for GUI (check with `echo $DISPLAY`)
DISPLAY=:0
```

### GUI Access (Gazebo, RViz)

```bash
# Allow Docker to access X11 display
xhost +local:docker

# Then docker compose up will show GUI on your monitor
```

## Learning Path

### Day 1: ROS Basics
- [ ] Understand nodes, topics, pub/sub
- [ ] Run example nodes in Docker
- [ ] Use `ros2 topic list`, `ros2 topic echo`

### Day 2: Build Camera Node
- [ ] Create a node that publishes test images
- [ ] Learn `sensor_msgs/Image` format
- [ ] Use `cv_bridge` to convert OpenCV ↔ ROS

### Day 3: Build Detection Node
- [ ] Subscribe to camera topic
- [ ] Run YOLOv8 inference
- [ ] Publish detection results

### Day 4: Integration & Review
- [ ] Run full pipeline
- [ ] Review ROS concepts for project
- [ ] Practice explaining the system

## ROS2 Cheat Sheet

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# See messages on a topic
ros2 topic echo /camera/image

# Topic info (message type, publishers, subscribers)
ros2 topic info /camera/image

# Message structure
ros2 interface show sensor_msgs/msg/Image

# Run a node
ros2 run <package> <node>

# Launch multiple nodes
ros2 launch <package> <launch_file.py>
```

---

## Gazebo Simulation (This Demo)

### What We Built

- **4-DOF robot arm** with camera on end-effector
- **Shelf world** with colored products (bottles, boxes, cans)
- **ros2_control** for joint position control
- Camera publishes to `/camera/image_raw` (same topic as webcam)

### Moving the Arm

```bash
# Move arm to look at shelf
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_joint], \
  points: [{positions: [0.0, 1.0, -1.5, 0.0], time_from_start: {sec: 2}}]}}"
```

**Joint positions in radians:**
| Joint | Axis | Range | Description |
|-------|------|-------|-------------|
| shoulder_pan | Z | ±3.14 | Rotate base left/right |
| shoulder_lift | Y | ±1.57 | Tilt forward/back |
| elbow | Y | ±2.5 | Bend elbow |
| wrist | Z | ±3.14 | Rotate wrist |

### ros2_control Architecture

```
Your command (action goal)
    ↓
joint_trajectory_controller (interpolates smooth path)
    ↓
ros2_control hardware interface
    ↓
gazebo_ros2_control plugin
    ↓
Gazebo physics simulation
    ↓
Joint states published to /joint_states
```

**Why ros2_control?**
- Industry standard for robot control
- Same interface for simulation AND real robot
- Just swap the hardware interface plugin

---

## ROS vs Gazebo vs Isaac Sim

These are different tools that work together:

| Tool | What it does | Analogy |
|------|--------------|---------|
| **ROS** | Communication between nodes | The "nervous system" |
| **Gazebo** | Physics simulation | The "virtual world" |
| **Isaac Sim** | NVIDIA's simulator (GPU-accelerated) | Premium "virtual world" |
| **RViz** | Visualization (no physics) | The "debug viewer" |

```
┌─────────────────────────────────────────────────────────────┐
│                    Gazebo (Physics Sim)                     │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
│  │   Robot     │  │   Shelf     │  │   Beverages         │  │
│  │   Model     │  │   Model     │  │   (physics objects) │  │
│  └─────────────┘  └─────────────┘  └─────────────────────┘  │
│         │                                                   │
│         │ Simulated sensors (camera, depth, lidar)          │
│         ▼                                                   │
│  ┌─────────────────────────────────────────────────────┐    │
│  │              ROS2 Topics                            │    │
│  │  /camera/image  /depth/image  /robot/joint_states   │    │
│  └─────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
              ┌───────────────────────────────┐
              │     Your Perception Node      │
              │  (same code as real robot!)   │
              └───────────────────────────────┘
```

**The magic:** Your perception code doesn't know if images come from real camera or Gazebo. Same code works in simulation and real robot.

### Gazebo on Mac / Docker

| Option | Works? | Notes |
|--------|--------|-------|
| Native Mac | ❌ | Not supported |
| Docker + X11 | ⚠️ | Possible but tricky (XQuartz) |
| Docker + VNC | ✅ | Works well, access via browser |
| Docker headless | ✅ | No GUI, but can save images/videos |

**For project:** Gazebo is optional. Focus on ROS basics first. If time permits, we can add Gazebo with VNC.

### Gazebo Setup (Optional - Stretch Goal)

We'll add to docker-compose.yml:
- Gazebo Fortress (works with ROS2 Humble)
- VNC server for GUI access
- Simple shelf + beverages world

```yaml
# Access Gazebo GUI at: http://localhost:6080
```

---

## retail robotics-Relevant Simulation Ideas

If we add Gazebo, we could simulate:

1. **Convenience store shelf** with beverage products
2. **TX SCARA robot arm** (or simplified version)
3. **Camera mounted on arm** publishing images
4. **Your perception node** detecting products
5. **Grasp planning** based on detections

This would be impressive to mention in practice: *"I built a ROS2 + Gazebo simulation of a shelf-stocking scenario to prepare for this role."*

---

## References

- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)
- [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [cv_bridge (ROS ↔ OpenCV)](https://github.com/ros-perception/vision_opencv)
- [Gazebo Sim](https://gazebosim.org/)
- [ROS2 + Gazebo Integration](https://gazebosim.org/docs/harmonic/ros2_integration)

---

## Appendix

### RQT Tools

**RQT** = ROS Qt GUI toolkit. Useful tools:

| Tool | Command | Purpose |
|------|---------|---------|
| Image viewer | `ros2 run rqt_image_view rqt_image_view` | View camera/detection images |
| Topic monitor | `ros2 run rqt_topic rqt_topic` | See all topics and messages |
| Node graph | `ros2 run rqt_graph rqt_graph` | Visualize node connections |
| Console | `ros2 run rqt_console rqt_console` | View logs from all nodes |

### Docker on Mac (Alternative Setup)

If you don't have a Linux PC with GPU, you can run on Mac with limitations:

**Docker on Mac = Linux inside a container.**

- ROS is built for Ubuntu - native Mac install is painful
- Docker runs a lightweight Linux environment
- Your Mac stays clean, ROS runs in isolated Linux container
- Industry standard - retail robotics likely uses Docker too
- Easy to delete and start fresh
- Reproducible environment (like Python venv but for entire OS)

```
┌─────────────────────────────────────┐
│           Your Mac (macOS)          │
│  ┌───────────────────────────────┐  │
│  │     Docker Container (Linux)  │  │
│  │  ┌─────────────────────────┐  │  │
│  │  │   ROS2 + PyTorch        │  │  │
│  │  │   Your perception nodes │  │  │
│  │  └─────────────────────────┘  │  │
│  └───────────────────────────────┘  │
└─────────────────────────────────────┘
```

**Limitations on Mac:**
- No NVIDIA GPU → CPU inference only (slower)
- GUI (Gazebo/RViz) requires XQuartz or VNC
- Apple Silicon (M1/M2) needs ARM images

**Recommendation:** Use Linux PC with NVIDIA GPU for best experience.
