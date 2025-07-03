# Franka Robot Joystick Control Setup

This system allows you to control a Franka robot using a joystick through ROS2 on your local machine, communicating with libfranka running on a realtime PC at **192.168.18.1**.

This setup requires 2 workstations. One local machine that interfaces with controller for input that runs a python ROS2 node, and a real-time PC that controls franka arm that runs a cpp client to receive instruction and implement control.

The cpp `franka_joystick_control_client` uses `libfranka` with cartesian impedance controller using custom trajectory points converted to `franka::CartesianPose` @ 1k Hz.

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

### 1. Realtime PC Setup (Assumed 192.168.18.1 for `[ROBOT_IP]`)

1. Connect to Franka Desk at `[ROBOT_IP]/desk/`
2. Activate FCI
3. Copy the built `robot client` to realtime PC (you will need to cmake and build `robot_client` first!)
4. run `./franka_joystick_control_client [ROBOT_IP]`

### 2. Local Machine Setup (Python Package)

1. **Build the package**:
   ```bash
   cd ~/ws
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
source [PATH_TO_JOYSTICK_TELEOP_WS]/install/setup.bash

# Launch with default settings
ros2 launch franka_joystick_control joystick_control.launch.py
```

## Joystick Controls

### Xbox Controller Layout:
- **D-PAD**: X/Y movement (horizontal plane)
- **Right Stick**: 
  - Vertical: Z movement (up/down)
  - Horizontal: Z rotation (yaw)
- **Left Stick**: 
  - Vertical: X rotation (row)
  - Horizontal: Y rotation (pitch)
- **A Button**: Reset pose to initial position
- **B Button**: Emergency stop

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

![joystick](https://github.com/user-attachments/assets/15f8e3d3-8715-4bac-8d14-873e2a712631)

## License
MIT
