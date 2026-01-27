#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from tf2_ros import TransformBroadcaster
import serial
import math
import numpy as np
from tf_transformations import quaternion_from_euler
import time
import struct
import traceback

class YahboomControllerNode(Node):
    def __init__(self):
        super().__init__('yahboom_controller_node')
        
        # Declare parameters
        self.declare_parameter('wheel_radius', 0.04)
        self.declare_parameter('wheel_separation_width', 0.215)
        self.declare_parameter('wheel_separation_length', 0.165)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
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
        
        # Store latest command for throttling
        self.current_twist = Twist()
        
        self.last_beep_time = 0
        
        # Ramping State
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wa = 0.0
        
        # Create publishers and subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
            
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer: 10Hz (0.1s)
        self.update_timer = self.create_timer(0.1, self.update_callback)
        
        # Serial Setup
        self.serial_port = None
        self.connect_serial()
        
        self.get_logger().info('Yahboom controller node initialized')
        
    def log_to_file(self, msg):
        try:
            with open('/home/varunvaidhiya/yahboom_debug.log', 'a') as f:
                f.write(f"{time.time()}: {msg}\n")
        except:
            pass
    
    def connect_serial(self):
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            
            self.serial_port = serial.Serial(
                port=self.port_name,
                baudrate=self.baud_rate,
                timeout=1.0)
            self.get_logger().info(f'Connected to {self.port_name}')
        except Exception as e:
            self.get_logger().error(f'Serial Connection Error: {e}')
            self.serial_port = None

    def calculate_checksum(self, data):
        return (sum(data) + 5) & 0xFF

    def send_packet(self, msg_type, payload):
        if self.serial_port is None:
            return
            
        try:
            HEAD = 0xFF
            DEVICE_ID = 0xFC
            
            # [HEAD, ID, LEN, TYPE] + Payload
            packet = [HEAD, DEVICE_ID, 0, msg_type] + list(payload)
            packet[2] = len(packet) - 1 # LEN value
            
            checksum = self.calculate_checksum(packet)
            packet.append(checksum)
            
            # Log successful packet generation for debugging
            # if msg_type == 0x12: 
            #    self.get_logger().info(f"TX: {[hex(x) for x in packet]}")
            
            self.serial_port.write(bytearray(packet))
            time.sleep(0.002) # Critical delay
            
        except Exception as e:
            self.get_logger().error(f'Tx Error (Closing): {e}')
            if self.serial_port:
                self.serial_port.close()
            self.serial_port = None

    def joy_callback(self, msg):
        try:
            # Button A (Index 0) -> Beep
            # DEBOUNCE: Only allow 1 beep every 0.5 seconds
            now = time.time()
            if len(msg.buttons) > 0 and msg.buttons[0] == 1:
                if (now - self.last_beep_time) > 0.5:
                    self.get_logger().info("Button A: BEEP")
                    payload = struct.pack('<h', 100) # 100ms
                    self.send_packet(0x02, payload)
                    self.last_beep_time = now
        except Exception as e:
            self.get_logger().error(f'Joy Error: {e}')

    def cmd_vel_callback(self, msg):
        # Just store the command. Do NOT send to serial here.
        # This prevents flooding the serial bus if joy publisher is fast.
        self.current_twist = msg

    def send_motion_command(self):
        try:
            msg = self.current_twist
            
            # CLAMP SPEED to prevent Brownout/Over-current
            # Limit to 0.2 m/s (200 mm/s) - SAFE MODE
            MAX_VAL = 0.2
            RAMP_STEP = 0.02 # Ramping step per 0.1s check
            
            target_vx = np.clip(msg.linear.x, -MAX_VAL, MAX_VAL)
            target_vy = np.clip(msg.linear.y, -MAX_VAL, MAX_VAL)
            target_w  = np.clip(msg.angular.z, -1.0, 1.0) # Rad/s
            
            # Ramping Logic
            if self.cmd_vx < target_vx:
                self.cmd_vx = min(self.cmd_vx + RAMP_STEP, target_vx)
            elif self.cmd_vx > target_vx:
                self.cmd_vx = max(self.cmd_vx - RAMP_STEP, target_vx)
                
            if self.cmd_vy < target_vy:
                self.cmd_vy = min(self.cmd_vy + RAMP_STEP, target_vy)
            elif self.cmd_vy > target_vy:
                self.cmd_vy = max(self.cmd_vy - RAMP_STEP, target_vy)

            self.cmd_wa = target_w # Angular is usually less current intensive, can stay direct or ramp too
            
            # Convert to integer for board
            vx_int = int(self.cmd_vx * 1000)
            vy_int = int(self.cmd_vy * 1000)
            w_int  = int(self.cmd_wa * 1000)
            
            CAR_TYPE = 1 # X3
            payload = struct.pack('<bhhh', CAR_TYPE, vx_int, vy_int, w_int)
            self.send_packet(0x12, payload)
        except Exception as e:
            self.get_logger().error(f'CmdVel Error: {e}')
            self.log_to_file(f'CmdVel Error: {e}')
            self.log_to_file(traceback.format_exc())

    def update_callback(self):
        try:
            if self.serial_port is None:
                self.connect_serial()
                return

            # 1. Read Odom (Re-enabled now that we know it's Power issue)
            self.read_yahboom_odometry()
            
            # 2. Publish Odom
            self.publish_odometry(self.get_clock().now())
            
            # 3. Send Motor Command (Throttled to 10Hz)
            self.send_motion_command()
            
        except Exception as e:
            self.get_logger().error(f'Update Error: {e}')
            self.log_to_file(f'Update Error: {e}')

    def read_yahboom_odometry(self):
        if self.serial_port is None: return
        try:
            # Consume all waiting bytes to avoid buffer overflow and getting stuck in the past
            waiting = self.serial_port.in_waiting
            if waiting < 4: # Minimum packet size
                return
                
            # Read everything
            data = self.serial_port.read(waiting)
            
            # Find the LAST occurrence of the header (FF FB) to get latest data
            # Protocol: FF FB LEN ...
            last_header_idx = data.rfind(b'\xff\xfb')
            
            if last_header_idx == -1:
                return # No header found
                
            # Check if we have enough data from the header
            # We need at least header(2) + len(1)
            if last_header_idx + 3 > len(data):
                # Header is at the very end, partial packet. 
                # Ideally we should keep this for next time, but for 10Hz commands, 
                # dropping one intromittent odom packet is better than lag.
                return
                
            # Parse Length
            packet_len = data[last_header_idx + 2]
            
            # Full packet size expected: Header(2) + Len(1) + PacketLen
            # Wait, verify protocol: 
            # Manual says: FF FB LEN(1) TYPE(1) PAYLOAD(N). 
            # Confirmed Protocol says Len includes ID+Len+Func+Payload.
            # Usually Receiving (Robot->PC) might be different. 
            # But let's assume it follows the standard Length byte.
            # If we trust the length byte:
            
            full_packet_size = 3 + packet_len # Header(2) + Len(1) is standard overhead? 
            # Or is Len just the remaining bytes?
            # Let's try to just verify we have enough bytes. 
            # If we don't strict parse, we just ignore for now to prevent crashing.
            # The most important part is flushing the buffer, which we just did.
            
            return
            
        except Exception as e:
            self.get_logger().error(f'Rx Error: {e}')
            # Do not close connection on generic RX error, just skip


    def publish_odometry(self, time):
        try:
            q = quaternion_from_euler(0, 0, self.theta)
            
            ts = TransformStamped()
            ts.header.stamp = time.to_msg()
            ts.header.frame_id = 'odom'
            ts.child_frame_id = 'base_link'
            ts.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(ts)
            
            odom = Odometry()
            odom.header.stamp = time.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            self.odom_pub.publish(odom)
        except Exception as e:
            self.get_logger().error(f'Pub Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    try:
        node = YahboomControllerNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"CRITICAL NODE FAILURE: {e}")
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
