from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.descriptions import ParameterValue
import os

def generate_launch_description():
    # Get the launch directory
    pkg_omnibot_description = get_package_share_directory('omnibot_description')
    xacro_file = os.path.join(pkg_omnibot_description, 'urdf', 'omnibot.urdf.xacro')
    
    # Process the xacro file using Command substitution
    robot_description_config = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Start the robot driver node (Yahboom Board)
    start_driver_node = Node(
        package='omnibot_driver',
        executable='yahboom_controller_node.py',
        name='yahboom_driver',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'baud_rate': 115200,
            'wheel_separation_x': 0.165,
            'wheel_separation_y': 0.215,
            'wheel_radius': 0.04
        }]
    )

    # Start the robot state publisher
    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_config}]
    )

    # Create and return launch description
    ld = LaunchDescription()
    ld.add_action(start_driver_node)
    ld.add_action(start_robot_state_publisher)

    return ld 