# 🤖 OmniBot - Autonomous Mecanum Wheel Robot with ROS Integration

[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-brightgreen)](https://docs.ros.org/en/jazzy/)
[![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04_LTS-orange)](https://releases.ubuntu.com/24.04/)
[![STM32](https://img.shields.io/badge/STM32-F407VG_Discovery-blue)](https://www.st.com/en/evaluation-tools/stm32f4discovery.html)

**Next-Gen Mobile Robotics Platform** | *Mecanum Mobility* | *ROS2 Powered* | *AI-Ready Architecture*

![Mecanum Robot Concept](https://via.placeholder.com/800x400.png?text=Mecanum+Robot+Demo+GIF+-+Add+Your+Project+Visuals+Here)

## 🚀 Project Vision
OmniBot is an open-source omnidirectional robotics platform designed for:
- **Advanced Research** in autonomous navigation and human-robot interaction
- **Seamless Integration** of foundation models for perception and planning
- **Educational Platform** for ROS2 and embedded systems development
- **Modular Foundation** for industrial-grade mobile robotics applications

## 🔑 Key Features
- **Holonomic Mobility**: 4WD mecanum wheels for 360° movement
- **Hybrid Control System**:
  - Raspberry Pi 5 (Ubuntu 24.04) for high-level computation
  - STM32F407VG Discovery board for real-time motor control
- **ROS2 Framework**:
  - Custom motor control package (`omnibot_driver`)
  - Sensor integration ready (LIDAR, RGB-D, IMU)
- **Future-Ready AI**:
  - Planned integration of vision transformers for perception
  - Natural language understanding for voice commands
  - Reinforcement learning for adaptive path planning

## 🛠️ Hardware Architecture
```
├── Compute Layer (RPi 5)
│   ├── Ubuntu 24.04 LTS
│   └── ROS2 Humble
├── Control Layer (STM32F407VG)
│   ├── L298N Motor Drivers (x2)
│   └── PWM-Controlled Mecanum Wheels
└── Perception Layer (Future)
    ├── Stereo Camera Setup
    └── 360° LIDAR Array
```

## ⚡ Getting Started

### Prerequisites
- Ubuntu 24.04 LTS (ARM64)
- ROS2 Jazzy 
- STM32CubeIDE 1.15.0+
- Python 3.10+

### Installation

Clone repository:
```bash
git clone https://github.com/varunvaidhiya/Mecanum-Wheel-Robot


cd Mecanum-Wheel_Robot

```

Install ROS dependencies:
```bash
sudo apt install ros-jazzy-robot-localization ros-jazzy-nav2-bringup
```

Build workspace:
```bash
colcon build --symlink-install
source install/setup.bash
```

Flash STM32 controller:
```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/omnibot_firmware.elf verify reset exit"
```

## 🧠 ROS2 Integration

Launch base control node:
```bash
ros2 launch omnibot_driver base_control.launch.py
```

Example velocity command:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

## 🌟 Future Roadmap

1. **Perception System** (Q2 2025)
   - Integration of Vision Transformer (ViT) models
   - 3D object detection pipeline

2. **Cognitive Layer** (Q3 2025)
   - Natural Language Processing (NLP) interface
   - LLM-based task understanding

3. **Autonomy Suite** (Q4 2025)
   - Neural Motion Planning (NMP)
   - Multi-modal sensor fusion

## 🤝 Contributing

We welcome contributions! Please see our [Contribution Guidelines](CONTRIBUTING.md) and:
1. Fork the repository
2. Create your feature branch:
   ```bash
   git checkout -b feature/amazing-feature
   ```
3. Commit your changes:
   ```bash
   git commit -m 'Add some amazing feature'
   ```
4. Push to the branch:
   ```bash
   git push origin feature/amazing-feature
   ```
5. Open a Pull Request


## 📧 Contact

**Project Maintainer**: [Varun Vaidhiya] - varun.vaidhiya@gmail.com
**Project Link**: [https://github.com/varunvaidhiya/Mecanum-Wheel-Robot](https://github.com/varunvaidhiya/Mecanum-Wheel-Robot)

## 🙏 Acknowledgments

- ROS2 Development Team
- STMicroelectronics for STM32 tools
- NVIDIA for AI framework inspiration
- OpenCV community for computer vision support.
