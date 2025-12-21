# OmniBot Mecanum Wheel Robot

A **ROS 2–based omnidirectional mobile robot platform** designed for **embodied AI research**, integrating real-world data collection, **Vision–Language–Action (VLA)** model inference, and human-in-the-loop control via an Android application.

This repository follows a **monorepo architecture**, hosting robot runtime software, data infrastructure, learning/inference pipelines, and user-facing interfaces in a single, well-structured codebase.

---

## Overview

OmniBot is a mecanum wheel robot system that supports:

* Omnidirectional mecanum wheel kinematics
* Multiple low-level motor control backends (STM32, Yahboom)
* ROS 2–based robot runtime and navigation stack
* **Structured data collection for VLA model fine-tuning**
* **Standalone VLA inference engine (edge or desktop)**
* Android tablet/phone app for monitoring, teleoperation, and voice control

The system is designed to scale from **manual teleoperation** to **learning-based autonomy**.

---

## High-Level Architecture

```
Android App
  │
  │  (ROSBridge / gRPC)
  ▼
Robot Runtime (ROS 2)
  ├── Control & Kinematics
  ├── Sensors & SLAM
  ├── Data Logging Hooks
  ▼
Data Engine
  ├── Episode-based datasets
  ├── Sensor + action alignment
  ▼
VLA Engine
  ├── Fine-tuning
  ├── Inference (policy server)
  └── Edge deployment
```

---

## Repository Structure

```
mecanum-vla-robot/
│
├── robot_ws/                     # ROS 2 runtime workspace
│   ├── src/
│   │   ├── omnibot_driver/        # Motor drivers (STM32 / Yahboom)
│   │   ├── omnibot_control/       # Controllers, planners
│   │   ├── omnibot_slam/          # SLAM and localization
│   │   ├── omnibot_perception/    # Cameras, depth, sensors
│   │   ├── omnibot_logging/       # Data hooks → Data Engine
│   │   └── omnibot_bridge/        # ROS ↔ external interfaces
│   └── launch/
│
├── data_engine/                  # Robot data collection & datasets
│   ├── schema/                   # Episode and sensor schemas
│   ├── collectors/               # rosbag → structured data
│   ├── labeling/                 # Optional annotation tools
│   ├── storage/                  # Parquet / video / numpy
│   └── tools/
│
├── vla_engine/                   # Vision–Language–Action stack
│   ├── models/                   # OpenVLA + adapters
│   ├── training/                 # Fine-tuning pipelines
│   ├── inference/                # Policy server / runtime
│   ├── deployment/               # ONNX / TensorRT / edge
│   └── benchmarks/
│
├── android_app/                  # Android monitoring & control app
│   ├── app/
│   ├── telemetry/
│   ├── voice/
│   └── slam_view/
│
├── infra/                        # DevOps & tooling
│   ├── docker/
│   ├── devcontainers/
│   └── ci/
│
├── docs/                         # System documentation
│   ├── architecture.md
│   ├── data_format.md
│   ├── inference_pipeline.md
│   └── android_protocol.md
│
└── README.md
```

---

## Hardware Support

### STM32 Microcontroller (Legacy)

* Direct PWM-based motor control
* External encoder feedback
* Custom serial communication protocol
* Requires firmware flashing

### Yahboom ROS Robot Expansion Board (Recommended)

* Integrated motor drivers and encoders
* USB-based communication
* Simplified command protocol
* Plug-and-play setup
* Based on the Yahboom ROS Driver Board

---

## Robot Runtime (ROS 2)

The ROS 2 workspace (`robot_ws/`) is responsible for:

* Low-level motor control
* Kinematics and motion control
* Sensor drivers and SLAM
* Publishing robot state and telemetry
* Logging synchronized data for learning

### Supported ROS 2 Distributions

* Humble (recommended)
* Iron or later (experimental)

---

## Data Engine

The Data Engine enables **learning-ready dataset creation** from real robot operation.

### Key Concepts

* Episode-based logging (not raw rosbag only)
* Time-synchronized sensor, state, and action streams
* Explicit action representations for VLA training

Example episode layout:

```
episode_00042/
├── rgb_front.mp4
├── depth_front.npy
├── joint_states.parquet
├── cmd_vel.parquet
├── action_tokens.npy
├── metadata.json
```

---

## VLA Engine

The VLA Engine is a **standalone learning and inference stack**, decoupled from ROS.

### Capabilities

* Fine-tuning VLA models on collected robot data
* Running inference via a policy server
* Supporting edge (Jetson / ARM) and desktop GPUs
* Benchmarking latency and throughput

ROS nodes interact with the VLA engine as **clients**, not as embedded inference logic.

---

## Android Application

The Android app provides a human interface for the robot.

### Features

* Live camera and telemetry streaming
* SLAM map visualization
* Manual teleoperation
* Voice command input (intent → robot action)

### Responsibilities (by design)

* UI and user interaction only
* No robot logic or control policies embedded

---

## Installation (Robot Runtime)

### Prerequisites

* ROS 2 (Humble or later)
* Python 3.8+
* `pyserial`

```bash
pip install pyserial
```

### Build

```bash
cd robot_ws
colcon build
source install/setup.bash
```

---

## Usage (Yahboom Board)

```bash
ros2 launch omnibot_driver yahboom_base_control.launch.py
```

Send velocity commands:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.5}, angular: {z: 0.0}}"
```

---

## Design Goals

* Clean separation of control, data, and learning
* Reproducible robotics datasets
* Hardware-agnostic inference
* Scalable from teleop → autonomy
* Suitable for research and production prototyping

---

## License

MIT License

---

## Acknowledgments

* Yahboom for the ROS Robot Expansion Board
* ROS 2 community
* Open-source embodied AI research community
