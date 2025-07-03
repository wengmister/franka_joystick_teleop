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
   source [path_to_franka_joyostick_teleop]/install/setup.bash
   ros2 launch franka_joystick_control joystick_control.launch.py
   ```

That's it! Your joystick should now control the Franka robot over the ethernet connection.

## Network Protocol

The system uses a simple UDP protocol with space-separated values to stream all joystick inputs. The axes follow standard `joynode` mapping convention for generic controllers:

```
axes: 

-0 (left horizontal) 
-1 (left vertical) 
-2 (left trigger) 
-3 (right horizontal) 
-4 (right vertical) 
-5 (right trigger) 
-6 (dpad horizontal) 
-7 (dpad vertical) 
```

Example: `"0.1 -0.05 0.0 0.0 0.0 0.2 0.0 0.0 0 0"`

## Filtering scheme

Joystick input tends to be noisy and presents control deadzones. Additionally rapid release of joystick tends to create discontinuities. In order to improve the robustness and smoothness of control, the following filtering scheme are used in this project:

```
Joystick input
   - exponential smoothing
   - release control
   - deadzoning/zero crossing
   - clamping
Trajectory filtering
   - low pass filter
   - release damping
```

## Demo

## License
MIT