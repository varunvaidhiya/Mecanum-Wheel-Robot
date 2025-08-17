# OmniBot Mecanum Wheel Robot

A ROS2-based mecanum wheel robot project with support for both STM32 and Yahboom ROS Robot Expansion Board for low-level motor control.

## Overview

This project implements a mecanum wheel robot with ROS2, featuring:
- Mecanum wheel kinematics for omnidirectional movement
- Support for STM32 microcontroller (original implementation)
- **NEW**: Support for Yahboom ROS Robot Expansion Board
- URDF models for visualization
- Launch files for easy deployment

## Hardware Support

### STM32 Microcontroller (Original)
- Direct motor control via PWM signals
- Encoder feedback for odometry
- Serial communication protocol

### Yahboom ROS Robot Expansion Board (New)
- Built-in motor drivers and encoders
- Simplified communication protocol
- USB connection for easy setup
- Based on [Yahboom ROS Driver Board](https://www.yahboom.net/study/ROS-Driver-Board)

## Project Structure

```
Mecanum-Wheel-Robot/
├── src/
│   ├── omnibot_driver/          # Motor control and hardware interface
│   │   ├── scripts/
│   │   │   ├── mecanum_controller_node.py    # STM32 controller
│   │   │   ├── serial_bridge_node.py         # STM32 serial bridge
│   │   │   ├── yahboom_controller_node.py    # NEW: Yahboom controller
│   │   │   └── yahboom_test_node.py          # NEW: Yahboom test node
│   │   ├── launch/
│   │   │   ├── base_control.launch.py        # STM32 launch
│   │   │   ├── yahboom_base_control.launch.py # NEW: Yahboom launch
│   │   │   └── yahboom_test.launch.py        # NEW: Yahboom test launch
│   │   └── config/
│   │       ├── mecanum_params.yaml           # STM32 parameters
│   │       └── yahboom_params.yaml           # NEW: Yahboom parameters
│   ├── omnibot_description/     # URDF models and visualization
│   ├── omnibot_bringup/         # High-level launch files
│   └── omnibot_firmware/        # STM32 firmware (not needed for Yahboom)
```

## Installation

### Prerequisites
- ROS2 (Humble or later)
- Python 3.8+
- Serial communication library: `pip install pyserial`

### Building the Project
```bash
# Clone the repository
git clone <repository-url>
cd Mecanum-Wheel-Robot

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

## Usage

### Using Yahboom Board (Recommended)

1. **Connect the Yahboom board** via USB to your computer
2. **Test the connection**:
   ```bash
   ros2 launch omnibot_driver yahboom_test.launch.py
   ```

3. **Run the robot controller**:
   ```bash
   ros2 launch omnibot_driver yahboom_base_control.launch.py
   ```

4. **Control the robot**:
   ```bash
   # In another terminal, publish velocity commands
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
   ```

### Using STM32 (Legacy)

1. **Flash the STM32 firmware** (see `omnibot_firmware/` directory)
2. **Connect the STM32** via USB
3. **Run the robot controller**:
   ```bash
   ros2 launch omnibot_driver base_control.launch.py
   ```

## Configuration

### Yahboom Board Parameters

Edit `src/omnibot_driver/config/yahboom_params.yaml` to adjust:
- Robot physical dimensions
- Serial port settings
- Motor control parameters
- Encoder settings

### STM32 Parameters

Edit `src/omnibot_driver/config/mecanum_params.yaml` for STM32 configuration.

## Key Differences: STM32 vs Yahboom

| Feature | STM32 | Yahboom Board |
|---------|-------|---------------|
| **Connection** | USB/Serial | USB |
| **Motor Control** | Direct PWM | Built-in drivers |
| **Encoders** | External | Built-in |
| **Protocol** | Custom serial | Simplified commands |
| **Setup** | Requires firmware | Plug-and-play |
| **Commands** | `<FL,FR,RL,RR>` | `MOTOR,FL,FR,RL,RR` |

## Troubleshooting

### Yahboom Board Issues

1. **Port not found**: Check if the board is connected and recognized
   ```bash
   ls /dev/ttyUSB*
   ```

2. **Permission denied**: Add user to dialout group
   ```bash
   sudo usermod -a -G dialout $USER
   ```

3. **Wrong baud rate**: Verify the board's communication settings

### STM32 Issues

1. **Firmware not loaded**: Flash the STM32 with the provided firmware
2. **Serial communication**: Check port and baud rate settings

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test with both STM32 and Yahboom boards
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- [Yahboom](https://www.yahboom.net/) for the ROS Robot Expansion Board
- ROS2 community for the excellent robotics framework
