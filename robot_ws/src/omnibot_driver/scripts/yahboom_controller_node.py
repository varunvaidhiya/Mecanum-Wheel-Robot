#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import math
import numpy as np
from tf_transformations import quaternion_from_euler
import time

class YahboomControllerNode(Node):
    """
    ROS2 Node for controlling a mecanum wheel robot using Yahboom ROS Robot Expansion Board.
    
    This node subscribes to velocity commands and converts them to Yahboom board commands
    based on the mecanum wheel kinematics equations.
    """
    
    def __init__(self):
        super().__init__('yahboom_controller_node')
        
        # Declare parameters
        self.declare_parameter('wheel_radius', 0.04)  # 4 cm radius
        self.declare_parameter('wheel_separation_width', 0.215)  # 21.5 cm between left and right wheels
        self.declare_parameter('wheel_separation_length', 0.165)  # 16.5 cm between front and back wheels
        self.declare_parameter('serial_port', '/dev/ttyUSB0')  # Yahboom typically uses USB
        self.declare_parameter('baud_rate', 115200)
        
        # Get parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation_width = self.get_parameter('wheel_separation_width').value
        self.wheel_separation_length = self.get_parameter('wheel_separation_length').value
        self.port_name = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        
        # Robot state
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.theta = 0.0
        
        # Create publishers and subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create timer for periodic updates
        self.update_timer = self.create_timer(0.01, self.update_callback)  # 100 Hz
        
        # Setup serial communication
        try:
            self.serial_port = serial.Serial(
                port=self.port_name,
                baudrate=self.baud_rate,
                timeout=1.0)
            self.get_logger().info(f'Successfully connected to Yahboom board on {self.port_name}')
            
            # Initialize the Yahboom board
            self.initialize_yahboom_board()
            
        except serial.SerialException as e:
            self.serial_port = None
            self.get_logger().error(f'Failed to open serial port {self.port_name}: {str(e)}')
        
        self.get_logger().info('Yahboom controller node initialized')
    
    def initialize_yahboom_board(self):
        """
        Initialize the Yahboom ROS Robot Expansion Board.
        """
        if self.serial_port is None or not self.serial_port.is_open:
            return
        
        try:
            # Send initialization command to Yahboom board
            # Based on Yahboom documentation, we need to initialize the board
            init_command = "INIT\n"
            self.serial_port.write(init_command.encode('utf-8'))
            time.sleep(0.1)
            
            # Set motor control mode to mecanum
            mode_command = "MODE_MECANUM\n"
            self.serial_port.write(mode_command.encode('utf-8'))
            time.sleep(0.1)
            
            self.get_logger().info('Yahboom board initialized successfully')
            
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to initialize Yahboom board: {str(e)}')
    
    def cmd_vel_callback(self, msg):
        """
        Callback for velocity commands.
        
        Args:
            msg (Twist): Velocity command message
        """
        # Calculate wheel velocities based on mecanum kinematics
        front_left, front_right, rear_left, rear_right = self.calculate_wheel_velocities(msg)
        
        # Send velocities to the Yahboom board
        self.send_velocities_to_yahboom(front_left, front_right, rear_left, rear_right)
    
    def calculate_wheel_velocities(self, twist):
        """
        Calculate wheel velocities based on mecanum kinematics.
        
        Args:
            twist (Twist): Velocity command
            
        Returns:
            tuple: (front_left, front_right, rear_left, rear_right) wheel velocities
        """
        # Extract velocity components
        vx = twist.linear.x
        vy = twist.linear.y
        omega = twist.angular.z
        
        # Calculate wheel velocities using mecanum kinematics equations
        # For a mecanum wheel robot, the wheel velocities are calculated as follows:
        # FL = Vx - Vy - (L+W)*omega
        # FR = Vx + Vy + (L+W)*omega
        # RL = Vx + Vy - (L+W)*omega
        # RR = Vx - Vy + (L+W)*omega
        # Where L is half the wheel separation length and W is half the wheel separation width
        
        L = self.wheel_separation_length / 2.0
        W = self.wheel_separation_width / 2.0
        k = L + W  # Distance from center to wheel (for rotation component)
        
        front_left = vx - vy - k * omega
        front_right = vx + vy + k * omega
        rear_left = vx + vy - k * omega
        rear_right = vx - vy + k * omega
        
        # Convert linear velocities to angular velocities (rad/s)
        front_left /= self.wheel_radius
        front_right /= self.wheel_radius
        rear_left /= self.wheel_radius
        rear_right /= self.wheel_radius
        
        return front_left, front_right, rear_left, rear_right
    
    def send_velocities_to_yahboom(self, front_left, front_right, rear_left, rear_right):
        """
        Send wheel velocities to the Yahboom board.
        
        Args:
            front_left (float): Front left wheel velocity
            front_right (float): Front right wheel velocity
            rear_left (float): Rear left wheel velocity
            rear_right (float): Rear right wheel velocity
        """
        if self.serial_port is None or not self.serial_port.is_open:
            self.get_logger().warning('Serial port not open, cannot send velocities')
            return
        
        # Yahboom board expects motor commands in a specific format
        # Based on Yahboom documentation, the format is typically:
        # "MOTOR,FL,FR,RL,RR\n" where values are PWM percentages (-100 to 100)
        
        # Convert rad/s to PWM percentage (-100 to 100)
        # This is a simplified conversion - you may need to tune these values
        max_rad_s = 10.0  # Maximum expected rad/s
        pwm_scale = 100.0 / max_rad_s
        
        fl_pwm = int(np.clip(front_left * pwm_scale, -100, 100))
        fr_pwm = int(np.clip(front_right * pwm_scale, -100, 100))
        rl_pwm = int(np.clip(rear_left * pwm_scale, -100, 100))
        rr_pwm = int(np.clip(rear_right * pwm_scale, -100, 100))
        
        command = f"MOTOR,{fl_pwm},{fr_pwm},{rl_pwm},{rr_pwm}\n"
        
        try:
            self.serial_port.write(command.encode('utf-8'))
            self.get_logger().debug(f'Sent motor command: {command.strip()}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to send velocities: {str(e)}')
    
    def update_callback(self):
        """
        Timer callback for periodic updates.
        """
        # Read odometry data from the Yahboom board
        self.read_yahboom_odometry()
        
        current_time = self.get_clock().now()
        self.publish_odometry(current_time)
    
    def read_yahboom_odometry(self):
        """
        Read odometry data from the Yahboom board.
        """
        if self.serial_port is None or not self.serial_port.is_open:
            return
        
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line:
                    self.process_yahboom_data(line)
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to read from Yahboom board: {str(e)}')
    
    def process_yahboom_data(self, data):
        """
        Process data received from the Yahboom board.
        
        Args:
            data (str): Data received from the Yahboom board
        """
        # Parse odometry data from Yahboom board
        # Expected format: "ODOM,x,y,theta\n"
        if data.startswith("ODOM,"):
            try:
                parts = data.split(',')
                if len(parts) >= 4:
                    self.x_pos = float(parts[1])
                    self.y_pos = float(parts[2])
                    self.theta = float(parts[3])
                    self.get_logger().debug(f'Updated odometry: x={self.x_pos}, y={self.y_pos}, theta={self.theta}')
            except ValueError as e:
                self.get_logger().warning(f'Invalid odometry data: {data}')
        
        # Parse encoder data
        elif data.startswith("ENCODERS,"):
            try:
                parts = data.split(',')
                if len(parts) >= 5:
                    fl_ticks = int(parts[1])
                    fr_ticks = int(parts[2])
                    rl_ticks = int(parts[3])
                    rr_ticks = int(parts[4])
                    # TODO: Use encoder data for more accurate odometry
                    self.get_logger().debug(f'Encoder ticks: FL={fl_ticks}, FR={fr_ticks}, RL={rl_ticks}, RR={rr_ticks}')
            except ValueError as e:
                self.get_logger().warning(f'Invalid encoder data: {data}')
    
    def publish_odometry(self, time):
        """
        Publish odometry information.
        
        Args:
            time (Time): Current time
        """
        # Create quaternion from yaw
        q = quaternion_from_euler(0, 0, self.theta)
        
        # Create and publish transform
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = time.to_msg()
        transform_stamped.header.frame_id = 'odom'
        transform_stamped.child_frame_id = 'base_link'
        transform_stamped.transform.translation.x = self.x_pos
        transform_stamped.transform.translation.y = self.y_pos
        transform_stamped.transform.translation.z = 0.0
        transform_stamped.transform.rotation.x = q[0]
        transform_stamped.transform.rotation.y = q[1]
        transform_stamped.transform.rotation.z = q[2]
        transform_stamped.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(transform_stamped)
        
        # Create and publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Set position
        odom_msg.pose.pose.position.x = self.x_pos
        odom_msg.pose.pose.position.y = self.y_pos
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        
        # Publish odometry message
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YahboomControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
