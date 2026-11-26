# Vision-Guided Autonomous Patrol Robot (ROS 2 Humble + Nav2 + YOLOv8)

An autonomous mobile robotics project built with **ROS 2 Humble**, **Nav2**, **Gazebo**, and **YOLOv8** on the **TurtleBot3 Waffle Pi** platform. The robot continuously patrols a mapped environment using LiDAR-based path planning while running real-time onboard object detection to pause when a person enters its field of view and automatically resume once the path clears.

---

## System Architecture

The project decouples **geometric obstacle avoidance** (LiDAR + Nav2) from **semantic perception** (RGB camera + YOLOv8) across two modular `rclpy` nodes:

```text
                       ┌──────────────────────────┐
                       │   Gazebo (Waffle Pi)     │
                       │  LiDAR + RGB Camera      │
                       └────┬────────────────┬────┘
                            │ /scan          │ /camera/image_raw
                            ▼                ▼
┌───────────────────┐  ┌──────────┐    ┌────────────────────────┐
│   RViz2 Display   │◄─┤   Nav2   │    │      vision_node       │
│ Map + Costmaps +  │  │  Stack   │    │  (YOLOv8n + cv_bridge) │
│ /vision_debug/img │  └────▲─────┘    └───┬────────────────┬───┘
└───────────────────┘       │              │                │
                 /cmd_vel & │              │ /target_detected│ /vision_debug/image
                 Nav2 Goals │              ▼                ▼
                       ┌────┴───────────────────┐     ┌───────────┐
                       │      patrol_node       │     │   RViz2   │
                       │ (BasicNavigator Loop)  │     │  Overlay  │
                       └────────────────────────┘     └───────────┘
```

### Nodes & Topics

| Node | Subscribes To | Publishes To | Role |
| :--- | :--- | :--- | :--- |
| **`vision_node`** | `/camera/image_raw` (`sensor_msgs/Image`) | `/target_detected` (`std_msgs/Bool`)<br>`/vision_debug/image` (`sensor_msgs/Image`) | Runs `YOLOv8n` inference on incoming camera frames and publishes detection states plus live bounding-box overlays. |
| **`patrol_node`** | `/target_detected` (`std_msgs/Bool`) | Nav2 Action Server (`NavigateToPose`) | Cycles through arena waypoints via `BasicNavigator`, calling `cancelTask()` when a target is spotted and resuming once cleared. |

---

## Tech Stack

* **Middleware & Navigation:** ROS 2 Humble, Nav2 (`nav2_simple_commander`), AMCL Localization, SLAM Toolbox
* **Simulation & Visualization:** Gazebo Classic 11 (`turtlebot3_waffle_pi`), RViz2
* **Computer Vision & AI:** Python 3, Ultralytics YOLOv8 (`yolov8n.pt`), OpenCV, `cv_bridge`

---

## Setup & Installation

### 1. Prerequisites
* Ubuntu 22.04 LTS with **ROS 2 Humble** installed
* TurtleBot3 Gazebo and Navigation2 packages:
  ```bash
  sudo apt update
  sudo apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-navigation2 ros-humble-cv-bridge
  ```
* Python dependencies (with NumPy `<2` and `setuptools==68.0.0` for ROS 2 Humble compatibility):
  ```bash
  pip install "numpy<2" "setuptools==68.0.0" ultralytics
  ```

### 2. Clone & Build
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone [https://github.com/Aneenams/ros2-turtlebot3-patrol.git](https://github.com/Aneenams/ros2-turtlebot3-patrol.git) patrol_pkg
cd ~/ros2_ws
colcon build --packages-select patrol_pkg
source install/setup.bash
```

---

## Running the Simulation

Launch the system across three terminals:

**1. Start Gazebo with TurtleBot3 Waffle Pi:**
```bash
source /usr/share/gazebo/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**2. Start Nav2 & RViz2:**
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/my_map.yaml
```
*In RViz2, use **2D Pose Estimate** to align the robot with its starting position on the map, and add an **Image** display subscribed to `/vision_debug/image` to view live YOLOv8 bounding boxes.*

**3. Launch Vision & Autonomous Patrol:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch patrol_pkg patrol.launch.py
```

### Runtime Parameters (`vision_node`)
* `target_class` (default: `'person'`) — COCO object class that triggers a patrol pause
* `confidence_threshold` (default: `0.5`) — Minimum detection confidence score
* `frame_skip` (default: `5`) — Runs inference every *N*th frame to maintain real-time simulation speed on CPU