from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    pkg_omnibot_driver = get_package_share_directory('omnibot_driver')
    pkg_omnibot_description = get_package_share_directory('omnibot_description')

    # Start the robot driver node
    start_driver_node = Node(
        package='omnibot_driver',
        executable='omnibot_driver_node',
        name='omnibot_driver',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'baud_rate': 115200,
            'wheel_separation_x': 0.3,
            'wheel_separation_y': 0.3,
            'wheel_radius': 0.05
        }]
    )

    # Start the robot state publisher
    start_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(pkg_omnibot_description + '/urdf/omnibot.urdf.xacro').read()}]
    )

    # Create and return launch description
    ld = LaunchDescription()
    ld.add_action(start_driver_node)
    ld.add_action(start_robot_state_publisher)

    return ld 