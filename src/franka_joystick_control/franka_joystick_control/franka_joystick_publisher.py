#!/usr/bin/env python3
# franka_joystick_publisher.py - Python ROS2 node for joystick control

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import socket
import time
import math

class FrankaJoystickPublisher(Node):
    def __init__(self):
        super().__init__('franka_joystick_publisher')
        
        # Declare parameters
        self.declare_parameter('robot_pc_ip', '192.168.18.1')
        self.declare_parameter('robot_pc_port', 8888)
        
        # Smoothing parameters
        self.declare_parameter('smoothing_alpha', 0.15)  # Much lower = much more smoothing
        self.declare_parameter('max_rate_linear', 0.8)   # Much slower max change rate  
        self.declare_parameter('max_rate_angular', 1.0)  # Much slower max change rate
        self.declare_parameter('release_decel_factor', 0.02)  # Much slower deceleration on release
        
        self.robot_pc_ip = self.get_parameter('robot_pc_ip').value
        self.robot_pc_port = self.get_parameter('robot_pc_port').value
        self.smoothing_alpha = self.get_parameter('smoothing_alpha').value
        self.max_rate_linear = self.get_parameter('max_rate_linear').value
        self.max_rate_angular = self.get_parameter('max_rate_angular').value
        self.release_decel_factor = self.get_parameter('release_decel_factor').value
        
        self.get_logger().info(f'Connecting to robot PC at {self.robot_pc_ip}:{self.robot_pc_port}')
        self.get_logger().info(f'Smoothing: alpha={self.smoothing_alpha}, max_rates=[{self.max_rate_linear}, {self.max_rate_angular}]')
        
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
        
        # Smoothing state
        self.smoothed_values = {
            'axis0': 0.0, 'axis1': 0.0, 'axis2': 0.0, 'axis3': 0.0,
            'axis4': 0.0, 'axis5': 0.0, 'axis6': 0.0, 'axis7': 0.0
        }
        self.last_time = time.time()
        self.first_callback = True
        
        self.get_logger().info('Franka Joystick Publisher started with smoothing')
        self.get_logger().info('NEW CONTROL MAPPING:')
        self.get_logger().info('  D-pad (axes 6,7): Robot translation (forward/back, left/right)')
        self.get_logger().info('  Left stick (axes 0,1): End-effector orientation (yaw/pitch)')
        self.get_logger().info('  Right stick (axes 3,4): Z-movement + roll rotation')
        self.get_logger().info('  Triggers (axes 2,5): Currently unused')
        self.get_logger().info('  A button: Reset pose')
        self.get_logger().info('  B button: Emergency stop')
        self.get_logger().info('Raw 8-axis values are sent directly to robot client')
    
    def apply_deadzone(self, value):
        return 0.0 if abs(value) < self.DEADZONE else value
    
    def clamp(self, value, min_val, max_val):
        return max(min_val, min(max_val, value))
    
    def apply_smoothing_and_rate_limiting(self, raw_values):
        """Apply exponential smoothing and rate limiting to prevent discontinuities"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Skip smoothing on first callback or if dt is too large (e.g., after pause)
        if self.first_callback or dt > 0.1:
            self.smoothed_values = raw_values.copy()
            self.first_callback = False
            return self.smoothed_values
        
        smoothed = {}
        
        for key in raw_values:
            raw_val = raw_values[key]
            prev_smoothed = self.smoothed_values[key]
            
            # Detect if joystick is in deadzone (released)
            is_released = abs(raw_val) < self.DEADZONE
            was_active = abs(prev_smoothed) > self.DEADZONE * 0.5
            
            if is_released and was_active:
                # Joystick released - apply controlled deceleration
                new_val = prev_smoothed * (1.0 - self.release_decel_factor)
                if abs(new_val) < self.DEADZONE * 0.1:  # Close enough to zero
                    new_val = 0.0
            else:
                # Normal smoothing
                # First apply exponential smoothing
                smoothed_val = self.smoothing_alpha * raw_val + (1.0 - self.smoothing_alpha) * prev_smoothed
                
                # Then apply rate limiting
                # Use angular limits for orientation axes (0,1,3), linear limits for translation axes (4,6,7)
                is_angular_axis = key in ['axis0', 'axis1', 'axis3']  # yaw, pitch, roll
                max_change = (self.max_rate_angular if is_angular_axis else self.max_rate_linear) * dt
                change = smoothed_val - prev_smoothed
                limited_change = self.clamp(change, -max_change, max_change)
                new_val = prev_smoothed + limited_change
                
                # Additional safety: clamp maximum change per timestep to prevent hardware discontinuities
                max_timestep_change = 0.01  # Maximum 0.01 change per timestep (very conservative)
                final_change = self.clamp(new_val - prev_smoothed, -max_timestep_change, max_timestep_change)
                new_val = prev_smoothed + final_change
            
            smoothed[key] = new_val
        
        self.smoothed_values = smoothed
        return smoothed
    
    def joy_callback(self, msg):
        # Extract raw joystick axes (8 axes total as expected by robot client)
        raw_axes = [0.0] * 8  # Initialize 8 axes
        emergency_stop = reset_pose = False
        
        # Extract all 8 axes from joystick message, applying deadzone
        if len(msg.axes) >= 8:
            for i in range(8):
                raw_axes[i] = self.apply_deadzone(msg.axes[i])
        else:
            # Handle case where joystick has fewer than 8 axes
            for i in range(min(len(msg.axes), 8)):
                raw_axes[i] = self.apply_deadzone(msg.axes[i])
        
        # Check buttons
        if len(msg.buttons) > max([self.BUTTON_A, self.BUTTON_B]):
            emergency_stop = bool(msg.buttons[self.BUTTON_B])
            reset_pose = bool(msg.buttons[self.BUTTON_A])
        
        # Apply smoothing and rate limiting to raw axes
        raw_values = {
            'axis0': raw_axes[0], 'axis1': raw_axes[1], 'axis2': raw_axes[2], 'axis3': raw_axes[3],
            'axis4': raw_axes[4], 'axis5': raw_axes[5], 'axis6': raw_axes[6], 'axis7': raw_axes[7]
        }
        
        smoothed = self.apply_smoothing_and_rate_limiting(raw_values)
        
        # Send smoothed command with 8 axes
        self.send_command(smoothed['axis0'], smoothed['axis1'], smoothed['axis2'], smoothed['axis3'],
                         smoothed['axis4'], smoothed['axis5'], smoothed['axis6'], smoothed['axis7'],
                         emergency_stop, reset_pose)
        
        # Debug output - show raw axes for significant changes
        if (any(abs(smoothed[f'axis{i}']) > 0.01 for i in range(8)) or emergency_stop or reset_pose):
            self.get_logger().info(
                f'Axes: [{smoothed["axis0"]:.2f}, {smoothed["axis1"]:.2f}, {smoothed["axis2"]:.2f}, {smoothed["axis3"]:.2f}, '
                f'{smoothed["axis4"]:.2f}, {smoothed["axis5"]:.2f}, {smoothed["axis6"]:.2f}, {smoothed["axis7"]:.2f}] '
                f'E-Stop: {"YES" if emergency_stop else "NO"} '
                f'Reset: {"YES" if reset_pose else "NO"}'
            )
    
    def send_command(self, axis0, axis1, axis2, axis3, axis4, axis5, axis6, axis7, estop, reset):
        command = f'{axis0} {axis1} {axis2} {axis3} {axis4} {axis5} {axis6} {axis7} {int(estop)} {int(reset)}'
        
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