# Franka Robot Joystick Control Setup

This system allows you to control a Franka robot using a joystick through ROS2 on your local machine, communicating with libfranka running on a realtime PC at **192.168.18.1**.

## Architecture

```
Local Machine (ROS2)           Ethernet (UDP)        Realtime PC (192.168.18.1)
┌─────────────────┐                                  ┌──────────────────────┐
│   Joystick      │            ┌─────────┐           │  franka_joystick_    │
│      ↓          │            │ Command │           │  control             │
│ ROS2 joy_node   │ ────────── │ UDP     │ ────────→ │         ↓            │
│      ↓          │            │ Socket  │           │  Franka Robot        │
│ Python          │            └─────────┘           │  (libfranka)         │
│ Publisher       │                                  └──────────────────────┘
└─────────────────┘
```

## Setup Instructions

### 1. Realtime PC Setup (Assumed 192.168.18.1)

Under construction.

### 2. Local Machine Setup (Python Package)

Under construction

1. **Build the package**:
   ```bash
   cd ~/franka_ws
   colcon build --packages-select franka_joystick_control
   source install/setup.bash
   ```

## Usage

### 1. Start the Franka Controller (Realtime PC)

```bash
# SSH into your realtime PC
ssh user@192.168.18.1

# Run the controller (replace with your robot's IP - assumed 192.168.18.10)
./franka_joystick_control <robot_ip>
```

Example:
```bash
./franka_joystick_control 192.168.18.10
```

The controller will start listening on UDP port 8888 and display:
```
UDP server listening on port 8888
Connected to robot at 172.16.0.2
WARNING: Robot will move based on joystick input!
Press Enter to start control loop...
```

### 2. Start ROS2 Joystick Control (Local Machine)

**Simple launch (uses default IP 192.168.18.1)**:
```bash
# Source ROS2
source /opt/ros/humble/setup.bash
source ~/franka_ws/install/setup.bash

# Launch with default settings
ros2 launch franka_joystick_control joystick_control.launch.py
```

## Joystick Controls

### Xbox Controller Layout:
- **Left Stick**: X/Y movement (horizontal plane)
- **Right Stick**: 
  - Vertical: Z movement (up/down)
  - Horizontal: Z rotation (yaw)
- **Left Trigger**: X rotation (roll, negative direction)
- **Right Trigger**: X rotation (roll, positive direction)
- **A Button**: Reset pose to initial position
- **B Button**: Emergency stop

### Safety Features:
- **Deadzone**: 0.1 (prevents drift)
- **Movement limits**: ±50cm from initial position
- **Emergency stop**: Immediately stops robot motion
- **Collision detection**: Uses Franka's built-in collision detection

## Quick Start Summary

1. **On Realtime PC (192.168.18.1)**:
   ```bash
   ssh user@192.168.18.1
   ./franka_joystick_control <robot_ip>
   ```

2. **On Local Machine**:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/franka_ws/install/setup.bash
   ros2 launch franka_joystick_control joystick_control.launch.py
   ```

That's it! Your joystick should now control the Franka robot over the ethernet connection.

## Customization

### Modifying Control Mapping:
Edit the axis/button mappings in the joystick publisher:
```python
self.AXIS_LEFT_STICK_HORIZONTAL = 0  # Change these values
self.BUTTON_A = 0                    # for different controllers
```

### Adjusting Movement Speed:
```python
self.LINEAR_SCALE = 1.0   # Reduce for slower movement
self.ANGULAR_SCALE = 1.0  # Reduce for slower rotation
```

### Adding More Controls:
- Modify the UDP command format
- Add new button/axis mappings
- Extend the Franka controller to handle new commands

## Network Protocol

The system uses a simple UDP protocol with space-separated values:
```
"linear_x linear_y linear_z angular_x angular_y angular_z emergency_stop reset_pose"
```

Example: `"0.1 -0.05 0.0 0.0 0.0 0.2 0 0"`