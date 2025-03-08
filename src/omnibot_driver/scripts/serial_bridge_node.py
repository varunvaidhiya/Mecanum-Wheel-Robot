#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import threading
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class SerialBridgeNode(Node):
    """
    ROS2 Node for bridging communication between ROS and the STM32 microcontroller.
    
    This node handles the serial communication protocol between the Raspberry Pi
    and the STM32 microcontroller for the OmniBot mecanum wheel robot.
    """
    
    def __init__(self):
        super().__init__('serial_bridge_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('read_timeout', 0.1)  # seconds
        
        # Get parameters
        self.port_name = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.read_timeout = self.get_parameter('read_timeout').value
        
        # Create publishers and subscribers
        self.serial_pub = self.create_publisher(String, 'serial_data', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Initialize serial port
        self.serial_port = None
        self.connect_serial()
        
        # Create read thread
        self.read_thread = threading.Thread(target=self.read_serial)
        self.read_thread.daemon = True
        self.read_thread.start()
        
        # Create heartbeat timer
        self.heartbeat_timer = self.create_timer(1.0, self.send_heartbeat)
        
        self.get_logger().info('Serial bridge node initialized')
    
    def connect_serial(self):
        """
        Connect to the serial port.
        """
        try:
            self.serial_port = serial.Serial(
                port=self.port_name,
                baudrate=self.baud_rate,
                timeout=self.read_timeout)
            self.get_logger().info(f'Successfully connected to serial port {self.port_name}')
            return True
        except serial.SerialException as e:
            self.serial_port = None
            self.get_logger().error(f'Failed to open serial port {self.port_name}: {str(e)}')
            return False
    
    def cmd_vel_callback(self, msg):
        """
        Callback for velocity commands.
        
        Args:
            msg (Twist): Velocity command message
        """
        if self.serial_port is None or not self.serial_port.is_open:
            self.get_logger().warning('Serial port not open, cannot send velocity command')
            return
        
        # Format the command for the STM32
        # Format: "<CMD_VEL,x,y,z>\n"
        command = f'<CMD_VEL,{msg.linear.x:.4f},{msg.linear.y:.4f},{msg.angular.z:.4f}>\n'
        
        try:
            self.serial_port.write(command.encode('utf-8'))
            self.get_logger().debug(f'Sent command: {command.strip()}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to send command: {str(e)}')
            # Try to reconnect
            self.connect_serial()
    
    def read_serial(self):
        """
        Thread function to continuously read from the serial port.
        """
        while rclpy.ok():
            if self.serial_port is None or not self.serial_port.is_open:
                # Try to reconnect every second
                time.sleep(1.0)
                self.connect_serial()
                continue
            
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if line:
                        # Publish the received data
                        msg = String()
                        msg.data = line
                        self.serial_pub.publish(msg)
                        self.get_logger().debug(f'Received: {line}')
                        
                        # Process the data if needed
                        self.process_serial_data(line)
                else:
                    # Small sleep to prevent CPU hogging
                    time.sleep(0.01)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {str(e)}')
                # Try to reconnect
                self.serial_port = None
                time.sleep(1.0)
                self.connect_serial()
    
    def process_serial_data(self, data):
        """
        Process data received from the serial port.
        
        Args:
            data (str): Data received from the serial port
        """
        # Example: Parse encoder data, sensor readings, etc.
        # Format: "<TYPE,value1,value2,...>\n"
        
        # Strip < and > characters
        if data.startswith('<') and data.endswith('>'):
            data = data[1:-1]
            parts = data.split(',')
            
            if len(parts) > 0:
                data_type = parts[0]
                
                if data_type == 'ENCODERS' and len(parts) >= 5:
                    # Process encoder data
                    try:
                        fl = float(parts[1])
                        fr = float(parts[2])
                        rl = float(parts[3])
                        rr = float(parts[4])
                        self.get_logger().debug(f'Encoder values: FL={fl}, FR={fr}, RL={rl}, RR={rr}')
                        # TODO: Use encoder data for odometry
                    except ValueError:
                        self.get_logger().warning(f'Invalid encoder data: {data}')
                
                elif data_type == 'STATUS':
                    # Process status message
                    self.get_logger().info(f'STM32 status: {",".join(parts[1:])}')
    
    def send_heartbeat(self):
        """
        Send a heartbeat message to the STM32.
        """
        if self.serial_port is None or not self.serial_port.is_open:
            return
        
        try:
            self.serial_port.write(b'<HEARTBEAT>\n')
        except serial.SerialException:
            # Try to reconnect
            self.connect_serial()

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()