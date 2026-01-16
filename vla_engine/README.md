# OmniBot VLA Engine

This directory contains the **offline** training and inference stack for the Vision-Language-Action models.

## Distinction
- **`vla_engine/` (This folder)**: Pure PyTorch/TensorFlow code for training models, running benchmarks, and defining model architectures. Independent of ROS.
- **`robot_ws/src/omnibot_vla/`**: The ROS 2 Node that wraps this engine for real-time robot control.

## Setup
```bash
pip install torch transformers accelerate
```
