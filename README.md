# ROS2 Perception Pipeline Demo

A complete perception-to-grasp pipeline for retail robotics, built with ROS2 Humble.

**Author:** Jin Wei Lim
**Purpose:** 

## What This Demo Shows

```
Camera → Detection → Tracking → Depth → Pose → Grasp
   ✅        ✅         ✅        ✅      ✅      ✅
```

| Component | Implementation | Description |
|-----------|----------------|-------------|
| Detection | YOLOv8 + fine-tuned model | Detects beverages (27 classes) |
| Tracking | BoT-SORT / ByteTrack | Persistent object IDs with trajectory trails |
| Depth | MiDaS monocular | Relative depth estimation |
| Pose | Geometry-based | 3D position from 2D + depth |
| Grasp | Heuristic | Grasp point prediction (side/top) |
| Simulation | Gazebo + ros2_control | 4-DOF arm with camera |

---

## Full System Capabilities

### Implementation Status

| Capability | Status | Notes |
|------------|--------|-------|
| **Inference Pipeline** | ✅ Done | Real-time detection → grasp |
| **Tracking** | ✅ Done | BoT-SORT with persistent IDs |
| **Depth Estimation** | ✅ Done | MiDaS monocular |
| **Pose Estimation** | ✅ Done | Geometry-based (swappable) |
| **Grasp Prediction** | ✅ Done | Heuristic (swappable) |
| **Simulation** | ✅ Done | Gazebo with realistic models |
| **ROS2 Integration** | ✅ Done | Modular node architecture |
| **Visualization** | ✅ Done | Multi-stream overlay |
| **Fine-tuned Model** | ✅ Done | 27 beverage classes |
| **Training Pipeline** | ❌ Not implemented | See PyTorch demo |
| **Data Annotation** | ❌ Not implemented | Would use Roboflow/CVAT |
| **Synthetic Data Gen** | ❌ Not implemented | Could add to Gazebo |
| **Motion Planning** | ❌ Not implemented | Would use MoveIt2 |
| **Grasp Execution** | ❌ Not implemented | Would add gripper control |
| **Failure Recovery** | ❌ Not implemented | Would add error handling |

---

## Production System Architecture

A complete retail robotics system has multiple phases:

### Phase 1: Training Pipeline (Offline)

```
┌─────────────────────────────────────────────────────────────────┐
│                     TRAINING PIPELINE                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌──────────────────┐    ┌──────────────────┐                  │
│  │  Synthetic Data  │    │    Real Data     │                  │
│  │  (Simulation)    │    │  (Store Images)  │                  │
│  └────────┬─────────┘    └────────┬─────────┘                  │
│           │                       │                             │
│           │  Domain Randomization │  Manual Annotation          │
│           │  - Random lighting    │  - Roboflow/CVAT            │
│           │  - Random placement   │  - Shop-provided images     │
│           │  - Random camera      │  - Edge cases               │
│           │                       │                             │
│           └───────────┬───────────┘                             │
│                       ▼                                         │
│              ┌─────────────────┐                                │
│              │  Train / Fine-  │                                │
│              │  tune YOLOv8    │                                │
│              └────────┬────────┘                                │
│                       ▼                                         │
│              ┌─────────────────┐                                │
│              │  Validate on    │                                │
│              │  Test Set       │                                │
│              └────────┬────────┘                                │
│                       ▼                                         │
│              ┌─────────────────┐                                │
│              │  Export Model   │ → cold_drinks.pt               │
│              └─────────────────┘                                │
│                                                                 │
│  Status: ❌ Not in ROS demo (see PyTorch demo for fine-tuning) │
└─────────────────────────────────────────────────────────────────┘
```

**Data Sources:**
| Source | Method | Ground Truth |
|--------|--------|--------------|
| Simulation (Gazebo/Isaac) | Render with domain randomization | Auto-labeled (free) |
| Store images | Shop provides photos | Manual annotation |
| CAD models | Manufacturer provides | Perfect 3D models |
| On-robot collection | Robot captures during operation | Human labels |

### Phase 2: Deployment Pipeline (Per-Store Setup)

```
┌─────────────────────────────────────────────────────────────────┐
│                    DEPLOYMENT PIPELINE                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────────┐                                           │
│  │  Install Robot  │                                           │
│  └────────┬────────┘                                           │
│           ▼                                                     │
│  ┌─────────────────┐    ┌─────────────────┐                    │
│  │ Camera Calib    │    │ Robot-Camera    │                    │
│  │ (intrinsics)    │    │ Transform (TF)  │                    │
│  └────────┬────────┘    └────────┬────────┘                    │
│           └───────────┬──────────┘                              │
│                       ▼                                         │
│  ┌─────────────────────────────────┐                           │
│  │  Shelf Mapping                  │                           │
│  │  - Where are product locations? │                           │
│  │  - What goes where?             │                           │
│  └────────────────┬────────────────┘                           │
│                   ▼                                             │
│  ┌─────────────────────────────────┐                           │
│  │  Load Pre-trained Model         │                           │
│  │  (or fine-tune on store data)   │                           │
│  └────────────────┬────────────────┘                           │
│                   ▼                                             │
│  ┌─────────────────────────────────┐                           │
│  │  Test & Validate                │                           │
│  └─────────────────────────────────┘                           │
│                                                                 │
│  Status: ⚠️ Partial (simulation only, no real calibration)     │
└─────────────────────────────────────────────────────────────────┘
```

### Phase 3: Operation Pipeline (Live Working Loop)

```
┌─────────────────────────────────────────────────────────────────┐
│                    OPERATION PIPELINE                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                   PERCEPTION LOOP                        │   │
│  │  ┌────────┐  ┌─────────┐  ┌───────┐  ┌──────┐  ┌──────┐│   │
│  │  │ Camera │→ │ Detect  │→ │ Track │→ │ Pose │→ │Grasp ││   │
│  │  └────────┘  └─────────┘  └───────┘  └──────┘  └──────┘│   │
│  │                                                         │   │
│  │  Status: ✅ IMPLEMENTED                                 │   │
│  └────────────────────────────┬────────────────────────────┘   │
│                               ▼                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                   DECISION LOOP                          │   │
│  │                                                          │   │
│  │  Check inventory:                                        │   │
│  │  - Product missing? → Need restock                       │   │
│  │  - Product misplaced? → Need rearrange                   │   │
│  │  - Product fallen? → Need pickup                         │   │
│  │  - Shelf full? → No action                               │   │
│  │                                                          │   │
│  │  Status: ❌ NOT IMPLEMENTED                              │   │
│  └────────────────────────────┬────────────────────────────┘   │
│                               ▼                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                   EXECUTION LOOP                         │   │
│  │                                                          │   │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌─────────┐ │   │
│  │  │ Plan     │→ │ Execute  │→ │ Verify   │→ │ Update  │ │   │
│  │  │ Motion   │  │ Grasp    │  │ Success  │  │ State   │ │   │
│  │  └──────────┘  └──────────┘  └──────────┘  └─────────┘ │   │
│  │                                                          │   │
│  │  Status: ❌ NOT IMPLEMENTED (would use MoveIt2)         │   │
│  └────────────────────────────┬────────────────────────────┘   │
│                               ▼                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                   ERROR HANDLING                         │   │
│  │                                                          │   │
│  │  If grasp fails:                                         │   │
│  │  - Retry with adjusted pose                              │   │
│  │  - Try alternative grasp point                           │   │
│  │  - Request teleoperator assistance                       │   │
│  │                                                          │   │
│  │  If unknown object:                                      │   │
│  │  - Log for training data                                 │   │
│  │  - Request human classification                          │   │
│  │                                                          │   │
│  │  Status: ❌ NOT IMPLEMENTED                              │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Phase 4: Continuous Improvement (Ongoing)

```
┌─────────────────────────────────────────────────────────────────┐
│                 CONTINUOUS IMPROVEMENT                          │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │                   ACTIVE LEARNING                        │   │
│  │                                                          │   │
│  │  Deploy → Detect → Low Confidence? → Human Review        │   │
│  │                          ↓                               │   │
│  │                   High Confidence → Log for validation   │   │
│  │                                                          │   │
│  │  Collect failure cases → Retrain → Deploy update         │   │
│  │                                                          │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
│  New Products:                                                  │
│  1. Store provides images/CAD                                   │
│  2. Add to training data                                        │
│  3. Fine-tune model                                             │
│  4. OTA update to robot                                         │
│                                                                 │
│  Status: ❌ NOT IMPLEMENTED                                     │
└─────────────────────────────────────────────────────────────────┘
```

---

## Demo vs Production Comparison

| Aspect | This Demo | Production System |
|--------|-----------|-------------------|
| **Perception** | ✅ YOLOv8 + MiDaS | RGB-D sensor, FoundationPose |
| **Tracking** | ✅ BoT-SORT | Same, or transformer-based |
| **Grasp Planning** | ✅ Heuristic | Contact-GraspNet, Dex-Net |
| **Motion Planning** | ❌ Teleop only | MoveIt2 + collision avoidance |
| **Depth** | ✅ Monocular (relative) | RGB-D (metric) |
| **Training** | ❌ Pre-trained | Domain randomization + fine-tune |
| **Failure Recovery** | ❌ None | Retry + teleop fallback |
| **Multi-store Deploy** | ❌ N/A | Same model, per-store calibration |

---

## Quick Start

### Prerequisites
- Linux PC with NVIDIA GPU (tested: Pop!_OS, RTX 2060)
- Docker with NVIDIA Container Toolkit
- Git

### Setup

```bash
# Clone and setup
git clone <repo>
cd ros-perception-demo
cp .env.example .env

# Build and enter container
docker compose build
docker compose up -d
docker exec -it ros_demo bash

# Inside container: build
cd /ros_ws
colcon build --packages-select perception_demo robot_description
source install/setup.bash

# Allow X11 display (run on host, not in container)
xhost +local:docker
```

### Run Full Pipeline

```bash
# Terminal 1: Gazebo simulation
ros2 launch robot_description simulation.launch.py world:=realistic

# Terminal 2: Perception pipeline (all in background)
ros2 run perception_demo tracker_node --ros-args \
  -p model_path:=/ros_ws/models/cold_drinks.pt \
  -p camera_topic:=/camera/image_raw &
ros2 run perception_demo depth_node &
ros2 run perception_demo object_estimator_node &
ros2 run perception_demo grasp_node &
ros2 run perception_demo viz_node

# Terminal 3: Keyboard teleop for arm
ros2 run perception_demo teleop_node

# Terminal 4: Visualize
ros2 run rqt_image_view rqt_image_view
# Select: /viz/grid (3-up view) or /viz/overlay (combined)
```

### Teleop Controls

```
Q/W - Rotate base (shoulder pan)
X/Z - Tilt arm (shoulder lift)
S/A - Bend elbow
1/2 - Rotate wrist

SPACE - Reset to home
ESC   - Quit
```

---

## Architecture

### Node Pipeline

```
┌─────────────┐
│ camera_node │ (/camera/image_raw)
│ (Gazebo/USB)│
└──────┬──────┘
       │
       ├────────────────────┬─────────────────────────────┐
       ▼                    ▼                             ▼
┌──────────────┐    ┌────────────┐                 ┌───────────┐
│ tracker_node │    │ depth_node │                 │  viz_node │
│  (YOLOv8 +   │    │  (MiDaS)   │                 │           │
│  BoT-SORT)   │    └─────┬──────┘                 └───────────┘
└──────┬───────┘          │                              ▲
       │                  │ /depth/raw (32FC1)           │
       │ /tracks          │ /depth/image (viz)           │
       │                  │                              │
       └────────┬─────────┘                              │
                ▼                                        │
┌───────────────────────────────┐                        │
│    object_estimator_node      │                        │
│  (combines 2D + depth → 3D)   │                        │
│                               │                        │
│  Output: /objects             │────────────────────────┤
│  - 3D position                │                        │
│  - 3D orientation             │                        │
│  - 3D dimensions              │                        │
│  - class, confidence, ID      │                        │
└───────────────┬───────────────┘                        │
                │                                        │
                ▼                                        │
┌───────────────────────────────┐                        │
│         grasp_node            │                        │
│  (predicts grasp points)      │────────────────────────┘
│                               │   /grasps
│  Input: /objects only         │
│  Output: /grasps (PoseArray)  │
└───────────────────────────────┘
```

### Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | Image | Raw camera feed |
| `/tracks` | Detection2DArray | Tracked objects with 2D bbox + class |
| `/tracks/image` | Image | Annotated with boxes + trails |
| `/depth/raw` | Image (32FC1) | Raw depth values (float) |
| `/depth/image` | Image (BGR) | Depth visualization (colormap) |
| `/objects` | Detection3DArray | Complete 3D object state |
| `/objects/image` | Image | Object state visualization |
| `/grasps` | PoseArray | Grasp points + approach direction |
| `/grasps/image` | Image | Grasp visualization |
| `/viz/grid` | Image | Raw \| Depth \| Combined (3-up) |
| `/viz/overlay` | Image | Full resolution combined view |

### Message Types

| Message | Contents |
|---------|----------|
| `Detection2DArray` | 2D bounding boxes, class, confidence, track ID |
| `Detection3DArray` | 3D bbox (pose + size), class, confidence, track ID |
| `PoseArray` | List of 3D poses (position + orientation) |
| `Image (32FC1)` | Single-channel float depth values |
| `Image (BGR)` | Standard color image |

---

## Nodes

### perception_demo package

| Node | Description | Key Parameters |
|------|-------------|----------------|
| `camera_node` | Camera source | `source:=webcam\|sim` |
| `detector_node` | YOLO detection only | `model_path:=...` |
| `tracker_node` | Detection + tracking | `tracker:=botsort\|bytetrack`, `model_path:=...` |
| `depth_node` | Monocular depth | `method:=midas` |
| `object_estimator_node` | 3D object state estimation | `detection_topic:=...`, `depth_topic:=...` |
| `grasp_node` | Grasp prediction | `objects_topic:=/objects`, `gripper_max_width:=0.1` |
| `viz_node` | Combined visualization | `cell_width:=320` |
| `teleop_node` | Keyboard arm control | - |

### robot_description package

| File | Description |
|------|-------------|
| `urdf/robot.urdf.xacro` | 4-DOF arm with camera |
| `worlds/shelf.world` | Simple colored primitives |
| `worlds/shelf_realistic.world` | 3D models (coke cans, beer) |
| `config/controllers.yaml` | ros2_control config |
| `launch/simulation.launch.py` | Launch Gazebo + robot |

---

## Object Estimation & Grasp Prediction

### Object Estimator Node (object_estimator_node)

Combines 2D detection with depth to produce complete 3D object state.

```
Input:
  - /tracks (Detection2DArray) - 2D bounding boxes + class
  - /depth/raw (Image, 32FC1) - raw depth values

Output:
  - /objects (Detection3DArray) - complete 3D object state:
    - 3D position [X, Y, Z]
    - 3D orientation (quaternion)
    - 3D dimensions [width, height, depth]
    - class name, confidence, track ID

Method:
1. Get bbox center (u, v) in pixels
2. Sample depth at center (center-weighted average)
3. Back-project to 3D:
   X = (u - cx) * Z / fx
   Y = (v - cy) * Z / fy
   Z = depth
4. Estimate dimensions from bbox size + depth:
   width = pixel_width * Z / fx
   height = pixel_height * Z / fy
```

**Key design:** Single source of truth for 3D object state. Downstream nodes (grasp, planning) use this output.

### Grasp Node (grasp_node)

Predicts grasp points from 3D object state.

```
Input:  /objects (Detection3DArray) - has everything needed!
Output: /grasps (PoseArray) - grasp poses with approach direction

Method:
1. Check object dimensions (from /objects):
   - Tall & thin (h > 1.5w) → side grasp
   - Wide (w > gripper_max) → top grasp
2. Offset grasp point from object center:
   - Side: 40% from top
   - Top: 15% from top
3. Set approach direction (encoded in quaternion)
```

**Grasp types:**
```
    ┌─────────┐
    │  ████   │ ← top grasp (bottles with narrow neck)
    │  ████   │
    │  ●═══●  │ ← side grasp (cans, standard bottles)
    │  ████   │
    └─────────┘
```

**Grasp feasibility:** Shows "TOO WIDE" if object width > gripper_max_width.

### Data Flow (Clean Architecture)

```
tracker_node ──┐
               ├──→ object_estimator_node ──→ grasp_node
depth_node ────┘           │
                           ▼
                    Single /objects topic contains:
                    - Position (from depth)
                    - Dimensions (from bbox + depth)
                    - Class/ID (from tracker)
```

**Why this is better:**
- Grasp doesn't re-sample depth (no duplicate work)
- Single source of truth for 3D state
- Easy to swap object_estimator implementation
- Clean separation of concerns

### Production Alternatives

| Component | Demo | Production |
|-----------|------|------------|
| Depth | MiDaS (relative) | RGB-D sensor (Intel RealSense) |
| Object Estimation | Geometry heuristic | FoundationPose, PoseCNN, DenseFusion |
| Grasp | Bbox heuristic | Contact-GraspNet, Dex-Net, GraspNet |

**Key design:** "The architecture is modular - `object_estimator_node` can be swapped from simple geometry to FoundationPose without changing grasp_node at all."

---

## Gazebo Simulation

### World Options

```bash
# Simple (colored primitives - faster)
ros2 launch robot_description simulation.launch.py world:=simple

# Realistic (3D models - requires gazebo_models)
ros2 launch robot_description simulation.launch.py world:=realistic
```

### Setup Gazebo Models (for realistic world)

```bash
# Inside container - one time setup
cd /root
git clone https://github.com/osrf/gazebo_models.git .gazebo/models
```

### Robot Arm

4-DOF arm with camera mounted on end-effector:

| Joint | Axis | Range | Motion |
|-------|------|-------|--------|
| shoulder_pan | Z | ±3.14 rad | Rotate base |
| shoulder_lift | Y | ±1.57 rad | Tilt up/down |
| elbow | Y | ±2.5 rad | Bend |
| wrist | Z | ±3.14 rad | Rotate |

---

## Custom Model

The demo includes a fine-tuned YOLOv8 model for cold drinks:

```bash
# Use custom model
ros2 run perception_demo tracker_node --ros-args \
  -p model_path:=/ros_ws/models/cold_drinks.pt
```

**Classes:** 27 beverage types (Coca-Cola, Pepsi, Sprite, etc.)

---

## Project Structure

```
ros-perception-demo/
├── docker/
│   └── Dockerfile           # ROS2 + PyTorch + CUDA + Gazebo
├── models/
│   └── cold_drinks.pt       # Fine-tuned YOLO model
├── src/
│   ├── perception_demo/     # Python package
│   │   └── nodes/
│   │       ├── camera/      # Camera sources
│   │       ├── detector/    # YOLO detection
│   │       ├── tracker/     # BoT-SORT/ByteTrack
│   │       ├── depth/       # MiDaS depth
│   │       ├── pose/        # 6DoF pose estimation
│   │       ├── grasp/       # Grasp prediction
│   │       ├── viz/         # Combined visualization
│   │       └── teleop/      # Keyboard arm control
│   └── robot_description/   # Gazebo simulation
│       ├── urdf/            # Robot model
│       ├── worlds/          # Environments
│       ├── config/          # ros2_control
│       └── launch/          # Launch files
└── README.md
```

---

## Development Workflow

```
MacBook (code) → git push → Gitea → git pull → Linux PC (run)
                                                    │
                                              Docker + GPU
                                              ROS2 + Gazebo
```

**Rebuild after changes:**
```bash
# Code changes only
colcon build --packages-select perception_demo

# New files added (need clean rebuild)
rm -rf build/perception_demo install/perception_demo
colcon build --packages-select perception_demo
source install/setup.bash
```

---

## References

- [ROS2 Humble](https://docs.ros.org/en/humble/)
- [Ultralytics YOLOv8](https://docs.ultralytics.com/)
- [MiDaS Depth](https://github.com/isl-org/MiDaS)
- [Gazebo Classic](https://classic.gazebosim.org/)
- [ros2_control](https://control.ros.org/)
