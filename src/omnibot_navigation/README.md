# OmniBot Autonomous Navigation

This package provides autonomous navigation capabilities for the OmniBot mecanum wheel robot using ROS2 Navigation2 stack.

## Features

- **SLAM Mapping**: Real-time mapping using slam_toolbox
- **Autonomous Navigation**: Path planning and obstacle avoidance
- **Waypoint Navigation**: Predefined waypoint following
- **Robot Localization**: EKF-based state estimation
- **Mecanum Wheel Support**: Full omnidirectional movement

## Package Structure

```
omnibot_navigation/
├── config/
│   ├── nav2_params.yaml          # Navigation2 parameters
│   ├── robot_localization.yaml   # EKF localization parameters
│   ├── slam_toolbox_params.yaml  # SLAM parameters
│   └── waypoints.yaml            # Default waypoints
├── launch/
│   ├── autonomous_navigation.launch.py
│   ├── autonomous_robot.launch.py
│   ├── autonomous_with_waypoints.launch.py
│   └── slam_toolbox.launch.py
├── scripts/
│   ├── waypoint_navigator.py     # Waypoint navigation node
│   └── test_autonomous.py       # Autonomous testing script
└── maps/                        # Saved maps (created at runtime)
```

## Quick Start

### 1. Build the Package

```bash
cd ~/Mecanum-Wheel-Robot
colcon build --packages-select omnibot_navigation
source install/setup.bash
```

### 2. Launch Autonomous Robot

For basic autonomous navigation with SLAM:
```bash
ros2 launch omnibot_navigation autonomous_robot.launch.py
```

For autonomous navigation with waypoint following:
```bash
ros2 launch omnibot_navigation autonomous_with_waypoints.launch.py
```

### 3. Test Autonomous Operation

Run the autonomous test sequence:
```bash
ros2 run omnibot_navigation test_autonomous.py
```

## Configuration

### Navigation Parameters

The navigation system is configured through `config/nav2_params.yaml`. Key parameters include:

- **Robot Dimensions**: Wheel radius, separation distances
- **Velocity Limits**: Maximum linear and angular velocities
- **Costmap Parameters**: Obstacle detection and inflation
- **Planner Settings**: Path planning algorithms

### Waypoint Configuration

Edit `config/waypoints.yaml` to define navigation waypoints:

```yaml
waypoints:
  - name: "start"
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  # Add more waypoints...
```

## Usage

### Manual Navigation

1. Launch the autonomous robot:
   ```bash
   ros2 launch omnibot_navigation autonomous_robot.launch.py
   ```

2. Use RViz to set navigation goals:
   - Click "2D Nav Goal" in RViz
   - Click and drag to set target position and orientation

### Waypoint Navigation

1. Launch with waypoint navigation:
   ```bash
   ros2 launch omnibot_navigation autonomous_with_waypoints.launch.py
   ```

2. The robot will automatically follow the predefined waypoints

### SLAM Mapping

1. Launch SLAM mode:
   ```bash
   ros2 launch omnibot_navigation slam_toolbox.launch.py
   ```

2. Drive the robot around to build the map
3. Save the map when complete:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f my_map
   ```

## Hardware Requirements

### Required Sensors

- **Depth Camera**: Xbox Kinect V2 (for depth/pointcloud/LaserScan)
- **RGB Camera**: Logitech Webcam (for visual input)
- **IMU**: MPU9250 (for orientation/odometry)
- **Encoders**: For odometry (integrated in Yahboom board)

### Recommended Setup

- **Computer**: Raspberry Pi 5 8GB
- **Controller**: Yahboom ROS Robot Expansion Board

## Troubleshooting

### Common Issues

1. **Navigation not working**: Check if Nav2 stack is running
   ```bash
   ros2 node list | grep nav2
   ```

2. **No map**: Ensure SLAM is running and robot is moving
   ```bash
   ros2 topic echo /map
   ```

3. **Localization issues**: Check IMU and odometry data
   ```bash
   ros2 topic echo /odom
   ros2 topic echo /imu/data
   ```

### Debug Commands

- Check navigation status:
  ```bash
  ros2 topic echo /navigate_to_pose/_action/status
  ```

- Monitor costmaps:
  ```bash
  ros2 run rviz2 rviz2 -d config/navigation.rviz
  ```

- Check waypoint navigation:
  ```bash
  ros2 topic echo /waypoint_navigator/feedback
  ```

## Advanced Configuration

### Custom Waypoints

Add waypoints programmatically:
```python
# In your own node
from omnibot_navigation.scripts.waypoint_navigator import WaypointNavigator

navigator = WaypointNavigator()
navigator.add_waypoint("custom_point", x=2.0, y=1.0, yaw=0.0)
```

### Custom Navigation Behaviors

Modify `config/nav2_params.yaml` to customize:
- Path planning algorithms
- Obstacle avoidance behavior
- Recovery behaviors
- Velocity constraints

## Safety Features

- **Emergency Stop**: Press Ctrl+C to stop all navigation
- **Obstacle Avoidance**: Automatic obstacle detection and avoidance
- **Velocity Limits**: Configurable maximum speeds
- **Timeout Protection**: Navigation goals timeout after specified time

## Contributing

To add new autonomous features:
1. Create new nodes in `scripts/`
2. Add configuration in `config/`
3. Create launch files in `launch/`
4. Update this README

## License

MIT License - see LICENSE file for details.
