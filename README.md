# ROS Demo - Beverage Detection Pipeline

Learning ROS by building a perception pipeline for retail robotics's shelf-stocking robots.

## Goal

Build a ROS2 perception node that:
1. Subscribes to camera images (simulated)
2. Runs YOLOv8 object detection
3. Publishes detection results
4. (Bonus) Adds depth estimation

## Architecture

```
┌─────────────────┐     /camera/image      ┌──────────────────┐     /detections
│  Camera Node    │ ──────────────────────▶│  Perception Node │ ──────────────────▶
│  (publishes     │    sensor_msgs/Image   │  (YOLOv8 +       │    DetectionArray
│   test images)  │                        │   PyTorch)       │
└─────────────────┘                        └──────────────────┘
```

## Why Docker for ROS?

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
ros_demo/
├── README.md                 # This file
├── docker/
│   ├── Dockerfile           # ROS2 + PyTorch environment
│   └── docker-compose.yml   # Easy container management
├── src/
│   └── perception_pkg/      # Our ROS2 package
│       ├── perception_pkg/
│       │   ├── __init__.py
│       │   ├── detection_node.py    # Main perception node
│       │   └── camera_node.py       # Simulated camera
│       ├── package.xml
│       └── setup.py
├── data/
│   └── test_images/         # Test images (from beverage demo)
└── models/
    └── yolov8n.pt           # YOLO model weights
```

## Setup

### Prerequisites
- Docker Desktop installed ✓
- This repo cloned

### Build and Run

```bash
# Build the Docker image
docker-compose build

# Start the container
docker-compose up -d

# Enter the container
docker exec -it ros_demo bash

# Inside container: build the workspace
colcon build
source install/setup.bash

# Run the nodes
ros2 run perception_pkg camera_node &
ros2 run perception_pkg detection_node
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
