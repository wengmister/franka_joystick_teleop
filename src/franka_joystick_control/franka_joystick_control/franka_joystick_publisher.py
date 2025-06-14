#!/usr/bin/env python3
# franka_joystick_publisher.py - Python ROS2 node for joystick control

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import socket

class FrankaJoystickPublisher(Node):
    def __init__(self):
        super().__init__('franka_joystick_publisher')
        
        # Declare parameters
        self.declare_parameter('robot_pc_ip', '192.168.18.1')
        self.declare_parameter('robot_pc_port', 8888)
        
        self.robot_pc_ip = self.get_parameter('robot_pc_ip').value
        self.robot_pc_port = self.get_parameter('robot_pc_port').value
        
        self.get_logger().info(f'Connecting to robot PC at {self.robot_pc_ip}:{self.robot_pc_port}')
        
        # Setup networking
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Subscribe to joystick
        self.joy_sub = self.create_subscription(
            Joy, 'joy', self.joy_callback, 10)
        
        # Button/axis mappings (Xbox controller)
        self.AXIS_LEFT_STICK_HORIZONTAL = 0
        self.AXIS_LEFT_STICK_VERTICAL = 1
        self.AXIS_RIGHT_STICK_HORIZONTAL = 3
        self.AXIS_RIGHT_STICK_VERTICAL = 4
        self.AXIS_LEFT_TRIGGER = 2
        self.AXIS_RIGHT_TRIGGER = 5
        
        self.BUTTON_A = 0  # Reset pose
        self.BUTTON_B = 1  # Emergency stop
        
        # Scaling and deadzone
        self.LINEAR_SCALE = 1.0
        self.ANGULAR_SCALE = 1.0
        self.DEADZONE = 0.1
        
        self.get_logger().info('Franka Joystick Publisher started')
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left stick: X/Y movement')
        self.get_logger().info('  Right stick: Z movement / Z rotation')
        self.get_logger().info('  Triggers: X/Y rotation')
        self.get_logger().info('  A button: Reset pose')
        self.get_logger().info('  B button: Emergency stop')
    
    def apply_deadzone(self, value):
        return 0.0 if abs(value) < self.DEADZONE else value
    
    def joy_callback(self, msg):
        # Extract joystick values
        linear_x = linear_y = linear_z = 0.0
        angular_x = angular_y = angular_z = 0.0
        emergency_stop = reset_pose = False
        
        if len(msg.axes) > max([self.AXIS_LEFT_STICK_HORIZONTAL, self.AXIS_LEFT_STICK_VERTICAL,
                               self.AXIS_RIGHT_STICK_HORIZONTAL, self.AXIS_RIGHT_STICK_VERTICAL,
                               self.AXIS_LEFT_TRIGGER, self.AXIS_RIGHT_TRIGGER]):
            
            # Linear movement
            linear_x = self.apply_deadzone(msg.axes[self.AXIS_LEFT_STICK_HORIZONTAL]) * self.LINEAR_SCALE
            linear_y = self.apply_deadzone(msg.axes[self.AXIS_LEFT_STICK_VERTICAL]) * self.LINEAR_SCALE
            linear_z = self.apply_deadzone(msg.axes[self.AXIS_RIGHT_STICK_VERTICAL]) * self.LINEAR_SCALE
            
            # Angular movement
            angular_z = self.apply_deadzone(msg.axes[self.AXIS_RIGHT_STICK_HORIZONTAL]) * self.ANGULAR_SCALE
            
            # Triggers for X and Y rotation
            left_trigger = (msg.axes[self.AXIS_LEFT_TRIGGER] + 1.0) / 2.0
            right_trigger = (msg.axes[self.AXIS_RIGHT_TRIGGER] + 1.0) / 2.0
            angular_x = (right_trigger - left_trigger) * self.ANGULAR_SCALE
        
        # Check buttons
        if len(msg.buttons) > max([self.BUTTON_A, self.BUTTON_B]):
            emergency_stop = bool(msg.buttons[self.BUTTON_B])
            reset_pose = bool(msg.buttons[self.BUTTON_A])
        
        # Send command
        self.send_command(linear_x, linear_y, linear_z, 
                         angular_x, angular_y, angular_z,
                         emergency_stop, reset_pose)
        
        # Debug output
        if (abs(linear_x) > 0.01 or abs(linear_y) > 0.01 or abs(linear_z) > 0.01 or
            abs(angular_x) > 0.01 or abs(angular_y) > 0.01 or abs(angular_z) > 0.01 or
            emergency_stop or reset_pose):
            
            self.get_logger().info(
                f'Lin: [{linear_x:.2f}, {linear_y:.2f}, {linear_z:.2f}] '
                f'Ang: [{angular_x:.2f}, {angular_y:.2f}, {angular_z:.2f}] '
                f'E-Stop: {"YES" if emergency_stop else "NO"} '
                f'Reset: {"YES" if reset_pose else "NO"}'
            )
    
    def send_command(self, lx, ly, lz, ax, ay, az, estop, reset):
        command = f'{lx} {ly} {lz} {ax} {ay} {az} {int(estop)} {int(reset)}'
        
        try:
            self.sock.sendto(command.encode(), (self.robot_pc_ip, self.robot_pc_port))
        except Exception as e:
            self.get_logger().warn(f'Failed to send command: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    node = FrankaJoystickPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()