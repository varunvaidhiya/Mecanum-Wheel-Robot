# omnibot_vla

This package provides integration with the [OpenVLA](https://github.com/openvla/openvla) model for Vision-Language-Action control of the OmniBot.

## Overview

The `vla_node` inside this package acts as a bridge between ROS 2 and the OpenVLA model.
- **Input**: RGB Images (`/image_raw`) + Text Prompts (`/vla/prompt`)
- **Output**: Velocity commands (`/cmd_vel`)

## Hardware Requirements

**This node is designed to run on a Desktop PC with a dedicated NVIDIA GPU.**
- **GPU**: NVIDIA RTX 3090 / 4090 / 5060 Ti or better.
- **VRAM**: 16GB+ recommended (or 8GB with 4-bit quantization).
- **RAM**: 32GB+ system RAM.

## Dependencies

This package requires several Python libraries not included in standard ROS 2 desktop installations:

```bash
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip3 install transformers accelerate bitsandbytes protobuf scipy
```

## Usage

### 1. Launching
On your **Desktop PC**:
```bash
ros2 launch omnibot_vla vla_desktop.launch.py
```

### 2. Prompting
To tell the robot what to do, publish a string to the prompt topic:
```bash
ros2 topic pub --once /vla/prompt std_msgs/msg/String "data: 'Find the red cup'"
```

### 3. Parameters
You can configure the model loading in `launch/vla_desktop.launch.py`:
- `model_path`: Path or HuggingFace ID (default: `openvla/openvla-7b`)
- `load_in_4bit`: Set to `True` if you have limited VRAM (<16GB).
