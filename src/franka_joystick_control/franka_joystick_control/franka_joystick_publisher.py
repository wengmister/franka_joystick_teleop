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
            'linear_x': 0.0, 'linear_y': 0.0, 'linear_z': 0.0,
            'angular_x': 0.0, 'angular_y': 0.0, 'angular_z': 0.0
        }
        self.last_time = time.time()
        self.first_callback = True
        
        self.get_logger().info('Franka Joystick Publisher started with smoothing')
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left stick: X/Y movement')
        self.get_logger().info('  Right stick: Z movement / Z rotation')
        self.get_logger().info('  Triggers: X/Y rotation')
        self.get_logger().info('  A button: Reset pose')
        self.get_logger().info('  B button: Emergency stop')
    
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
                max_change = (self.max_rate_linear if 'linear' in key else self.max_rate_angular) * dt
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
        # Extract raw joystick values
        raw_linear_x = raw_linear_y = raw_linear_z = 0.0
        raw_angular_x = raw_angular_y = raw_angular_z = 0.0
        emergency_stop = reset_pose = False
        
        if len(msg.axes) > max([self.AXIS_LEFT_STICK_HORIZONTAL, self.AXIS_LEFT_STICK_VERTICAL,
                               self.AXIS_RIGHT_STICK_HORIZONTAL, self.AXIS_RIGHT_STICK_VERTICAL,
                               self.AXIS_LEFT_TRIGGER, self.AXIS_RIGHT_TRIGGER]):
            
            # Linear movement (apply deadzone to raw values)
            raw_linear_x = self.apply_deadzone(msg.axes[self.AXIS_LEFT_STICK_HORIZONTAL]) * self.LINEAR_SCALE
            raw_linear_y = self.apply_deadzone(msg.axes[self.AXIS_LEFT_STICK_VERTICAL]) * self.LINEAR_SCALE
            raw_linear_z = self.apply_deadzone(msg.axes[self.AXIS_RIGHT_STICK_VERTICAL]) * self.LINEAR_SCALE
            
            # Angular movement
            raw_angular_z = self.apply_deadzone(msg.axes[self.AXIS_RIGHT_STICK_HORIZONTAL]) * self.ANGULAR_SCALE
            
            # Triggers for X and Y rotation
            left_trigger = (msg.axes[self.AXIS_LEFT_TRIGGER] + 1.0) / 2.0
            right_trigger = (msg.axes[self.AXIS_RIGHT_TRIGGER] + 1.0) / 2.0
            raw_angular_x = (right_trigger - left_trigger) * self.ANGULAR_SCALE
        
        # Check buttons
        if len(msg.buttons) > max([self.BUTTON_A, self.BUTTON_B]):
            emergency_stop = bool(msg.buttons[self.BUTTON_B])
            reset_pose = bool(msg.buttons[self.BUTTON_A])
        
        # Apply smoothing and rate limiting
        raw_values = {
            'linear_x': raw_linear_x, 'linear_y': raw_linear_y, 'linear_z': raw_linear_z,
            'angular_x': raw_angular_x, 'angular_y': raw_angular_y, 'angular_z': raw_angular_z
        }
        
        smoothed = self.apply_smoothing_and_rate_limiting(raw_values)
        
        # Send smoothed command
        self.send_command(smoothed['linear_x'], smoothed['linear_y'], smoothed['linear_z'], 
                         smoothed['angular_x'], smoothed['angular_y'], smoothed['angular_z'],
                         emergency_stop, reset_pose)
        
        # Debug output - show both raw and smoothed for significant changes
        if (abs(smoothed['linear_x']) > 0.01 or abs(smoothed['linear_y']) > 0.01 or abs(smoothed['linear_z']) > 0.01 or
            abs(smoothed['angular_x']) > 0.01 or abs(smoothed['angular_y']) > 0.01 or abs(smoothed['angular_z']) > 0.01 or
            emergency_stop or reset_pose):
            
            # Check if there's significant smoothing happening
            max_diff = max(abs(raw_values[k] - smoothed[k]) for k in raw_values)
            if max_diff > 0.05:  # Show raw vs smoothed when there's significant difference
                self.get_logger().info(
                    f'Raw: [{raw_linear_x:.2f}, {raw_linear_y:.2f}, {raw_linear_z:.2f}] '
                    f'[{raw_angular_x:.2f}, {raw_angular_y:.2f}, {raw_angular_z:.2f}]'
                )
                self.get_logger().info(
                    f'Smoothed: [{smoothed["linear_x"]:.2f}, {smoothed["linear_y"]:.2f}, {smoothed["linear_z"]:.2f}] '
                    f'[{smoothed["angular_x"]:.2f}, {smoothed["angular_y"]:.2f}, {smoothed["angular_z"]:.2f}] '
                    f'E-Stop: {"YES" if emergency_stop else "NO"} Reset: {"YES" if reset_pose else "NO"}'
                )
            else:
                self.get_logger().info(
                    f'Lin: [{smoothed["linear_x"]:.2f}, {smoothed["linear_y"]:.2f}, {smoothed["linear_z"]:.2f}] '
                    f'Ang: [{smoothed["angular_x"]:.2f}, {smoothed["angular_y"]:.2f}, {smoothed["angular_z"]:.2f}] '
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