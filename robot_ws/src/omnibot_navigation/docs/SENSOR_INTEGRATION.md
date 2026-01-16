# Sensor Integration Guide for OmniBot Autonomous Navigation

This guide explains how to integrate sensors with the OmniBot for autonomous navigation.

## Required Sensors

### 1. Depth Camera (Essential)
**Purpose**: Obstacle detection, mapping, and localization (Pointcloud to LaserScan)

**Recommended Model**:
- **Xbox Kinect V2** (Depth/Pointcloud)
- Alternative: Intel RealSense D435

**Integration Steps**:
1. Connect Kinect V2 via USB 3.0
2. Install libfreenect2 and ros2 driver:
   ```bash
   # Install libfreenect2 dependencies
   sudo apt-get install build-essential cmake pkg-config
   sudo apt-get install libusb-1.0-0-dev
   sudo apt-get install libturbojpeg0-dev libglfw3-dev
   
   # Install ROS 2 Jazzy driver
   sudo apt install ros-jazzy-kinect-ros2
   ```
3. Launch Kinect node:
   ```bash
   ros2 launch kinect_ros2 kinect_ros2.launch.py
   ```

### 2. IMU (Essential)
**Purpose**: Orientation tracking, odometry enhancement

**Recommended Models**:
- MPU9250 (9-axis, ~$10)
- BNO055 (9-axis with fusion, ~$15)
- LSM9DS1 (9-axis, ~$8)

**Integration Steps**:
1. Connect IMU via I2C or SPI
2. Install IMU driver:
   ```bash
   sudo apt install ros-humble-imu-tools
   ```
3. Launch IMU node:
   ```bash
   ros2 run imu_tools imu_complementary_filter_node
   ```

### 3. RGB Camera (Essential)
**Purpose**: Visual navigation, object detection, VLA input

**Recommended Model**:
- **Logitech Webcam** (C920 or similar)

**Integration Steps**:
1. Connect Webcam via USB
2. Install USB Cam driver:
   ```bash
   sudo apt install ros-jazzy-usb-cam
   ```

## Hardware Setup

### Wiring Diagram
```
Raspberry Pi 5
├── USB 3.0 → Xbox Kinect V2 (Requires own power)
├── USB → Logitech Webcam
├── I2C → IMU (SDA/SCL)
├── USB → Yahboom Board
└── Power → 5V/5A (Pi), 12V (Motors/Kinect)
```

### Pin Connections (IMU)
```
IMU (MPU9250) → Raspberry Pi
VCC → 3.3V
GND → GND
SDA → GPIO 2 (Pin 3)
SCL → GPIO 3 (Pin 5)
```

## Software Integration

### 1. Install Required Packages
```bash
# Kinect support
sudo apt install ros-jazzy-kinect-ros2
sudo apt install ros-jazzy-depthimage-to-laserscan

# Webcam support
sudo apt install ros-jazzy-usb-cam

# IMU support
sudo apt install ros-jazzy-imu-tools
sudo apt install ros-jazzy-imu-filter-madgwick

# Navigation stack
sudo apt install ros-jazzy-navigation2
sudo apt install ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-robot-localization
```

### 2. Create Sensor Launch File
Create `src/omnibot_navigation/launch/sensors.launch.py`:

```python
#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Kinect node
    kinect_node = Node(
        package='kinect_ros2',
        executable='kinect_ros2_node',
        name='kinect_ros2',
        output='screen'
    )
    
    # Depth to LaserScan
    depth_to_laser_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan_node',
        remappings=[('depth', '/kinect/depth/image_raw'),
                    ('depth_camera_info', '/kinect/depth/camera_info')],
        parameters=[{
            'output_frame': 'kinect_link',
            'range_min': 0.5,
            'range_max': 4.5,
            'scan_height': 1
        }]
    )

    # Webcam node
    webcam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        parameters=[{
            'video_device': '/dev/video0',
            'framerate': 30.0,
            'image_width': 640,
            'image_height': 480
        }]
    )
    
    # IMU node
    imu_node = Node(
        package='imu_tools',
        executable='imu_complementary_filter_node',
        name='imu_complementary_filter_node',
        parameters=[{
            'use_mag': True,
            'publish_tf': True,
            'reverse_tf': False,
            'fixed_frame': 'odom',
            'imu_frame': 'imu_link'
        }],
        output='screen'
    )
    
    return LaunchDescription([
        kinect_node,
        depth_to_laser_node,
        webcam_node,
        imu_node
    ])
```

### 3. Update Navigation Launch
Modify `autonomous_robot.launch.py` to include sensors:

```python
# Add sensor launch
sensors = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(bringup_dir, 'launch', 'sensors.launch.py')),
    launch_arguments={'use_sim_time': use_sim_time}.items())
```

## Configuration

### Depth Camera to LaserScan Configuration
Edit `config/nav2_params.yaml`:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["voxel_layer", "inflation_layer"]
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan # Virtual scan from depthimage_to_laserscan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
```

### IMU Configuration
Edit `config/robot_localization.yaml`:

```yaml
ekf_filter_node:
  ros__parameters:
    imu0: /imu/data
    imu0_config: [false, false, false,
                  true, true, true,
                  false, false, false,
                  true, true, true,
                  true, true, true]
```

## Testing Sensors

### 1. Test Sensors
```bash
# Launch Sensors
ros2 launch omnibot_navigation sensors.launch.py

# Check LaserScan (from Kinect)
ros2 topic echo /scan

# Check Webcam
ros2 run rqt_image_view rqt_image_view
```

### 2. Test IMU
```bash
# Check IMU data
ros2 topic echo /imu/data

# Check IMU orientation
ros2 topic echo /imu/mag
```

### 3. Test Sensor Fusion
```bash
# Launch robot localization
ros2 run robot_localization ekf_node --ros-args --params-file config/robot_localization.yaml

# Check fused odometry
ros2 topic echo /odometry/filtered
```

## Troubleshooting

### Common Issues

1. **Kinect not detected**:
   ```bash
   # Check USB devices
   lsusb
   # Look for Microsoft Corp. Xbox NUI Sensor
   ```

2. **IMU not responding**:
   ```bash
   # Check I2C devices
   sudo i2cdetect -y 1
   
   # Enable I2C
   sudo raspi-config
   ```

3. **Sensor data not publishing**:
   ```bash
   # Check node status
   ros2 node list
   
   # Check topics
   ros2 topic list
   ```

### Performance Optimization

1. **Reduce Pointcloud density** for better performance if needed.

2. **Filter IMU data** to reduce noise:
   ```yaml
   imu:
     filter_frequency: 10.0  # Hz
   ```

## Advanced Integration

### Multiple Sensors
For advanced autonomous operation, consider:
- **Stereo cameras** for depth perception
- **Multiple LIDARs** for 360° coverage
- **GPS** for outdoor navigation
- **Encoders** for wheel odometry (already implemented)

### Sensor Calibration
1. **Camera calibration**: Ensure Kinect and Webcam intrinsic parameters are calibrated
2. **IMU calibration**: Run calibration sequence for accurate orientation
3. **Camera calibration**: Use checkerboard for camera calibration

## Safety Considerations

1. **Emergency stop**: Always have manual override capability
2. **Sensor redundancy**: Use multiple sensors for critical functions
3. **Fail-safe modes**: Implement safe behavior when sensors fail
4. **Testing**: Test in controlled environments before autonomous operation

## Next Steps

After sensor integration:
1. Test individual sensors
2. Test sensor fusion
3. Test SLAM mapping
4. Test autonomous navigation
5. Test waypoint following

For detailed configuration, see the main navigation documentation.
