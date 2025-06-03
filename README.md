# Franka Robot Joystick Control Setup (Python-Only)

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

### 1. Realtime PC Setup (192.168.18.1)

1. **SSH into your realtime PC**:
   ```bash
   ssh user@192.168.18.1
   ```

2. **Install libfranka** (if not already installed):
   ```bash
   sudo apt install libfranka-dev
   ```

3. **Create and compile the Franka controller**:
   ```bash
   # Copy the franka_joystick_control.cpp file to the realtime PC
   g++ -std=c++17 franka_joystick_control.cpp -lfranka -lpthread -o franka_joystick_control
   ```

### 2. Local Machine Setup (Python Package)

1. **Install ROS2** (Humble recommended) and joy package:
   ```bash
   sudo apt install ros-humble-desktop ros-humble-joy
   ```

2. **Create ROS2 workspace and package structure**:
   ```bash
   mkdir -p ~/franka_ws/src/franka_joystick_control
   cd ~/franka_ws/src/franka_joystick_control
   
   # Create the Python package structure:
   mkdir -p franka_joystick_control
   mkdir -p launch
   mkdir -p resource
   
   # Copy files to correct locations:
   # - setup.py to franka_joystick_control/ (root)
   # - package.xml to franka_joystick_control/ (root)  
   # - resource/franka_joystick_control to resource/ (empty marker file)
   # - franka_joystick_control/__init__.py to franka_joystick_control/ (package init)
   # - franka_joystick_publisher.py to franka_joystick_control/ (as module)
   # - joystick_control.launch.py to launch/
   ```

3. **Package directory structure should look like**:
   ```
   ~/franka_ws/src/franka_joystick_control/
   ├── setup.py
   ├── package.xml
   ├── resource/
   │   └── franka_joystick_control
   ├── franka_joystick_control/
   │   ├── __init__.py
   │   └── franka_joystick_publisher.py
   └── launch/
       └── joystick_control.launch.py
   ```

4. **Build the package**:
   ```bash
   cd ~/franka_ws
   colcon build --packages-select franka_joystick_control
   source install/setup.bash
   ```

### 3. Network Configuration

Your setup uses ethernet connection to **192.168.18.1**. 

1. **Test connectivity**:
   ```bash
   ping 192.168.18.1
   ```

2. **Check your local IP** (should be in same subnet):
   ```bash
   ip addr show
   # Should show something like 192.168.18.x
   ```

## Usage

### 1. Start the Franka Controller (Realtime PC)

```bash
# SSH into your realtime PC
ssh user@192.168.18.1

# Run the controller (replace with your robot's IP - typically something like 172.16.0.2)
./franka_joystick_control <robot_ip>
```

Example:
```bash
./franka_joystick_control 172.16.0.2
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

**Manual startup for debugging**:
```bash
# Terminal 1: Joy node
ros2 run joy joy_node

# Terminal 2: Python joystick publisher  
ros2 run franka_joystick_control franka_joystick_publisher
```

You should see output like:
```
[INFO] [franka_joystick_publisher]: Connecting to robot PC at 192.168.18.1:8888
[INFO] [franka_joystick_publisher]: Franka Joystick Publisher started
[INFO] [franka_joystick_publisher]: Controls:
[INFO] [franka_joystick_publisher]:   Left stick: X/Y movement
[INFO] [franka_joystick_publisher]:   Right stick: Z movement / Z rotation
[INFO] [franka_joystick_publisher]:   Triggers: X/Y rotation
[INFO] [franka_joystick_publisher]:   A button: Reset pose
[INFO] [franka_joystick_publisher]:   B button: Emergency stop
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

## Troubleshooting

### Common Issues:

1. **"Failed to bind socket"**:
   - Port 8888 already in use
   - Try a different port or kill existing processes

2. **"Failed to send command"**:
   - Check network connectivity
   - Verify IP address and port
   - Check firewall settings

3. **Robot doesn't move**:
   - Verify joystick is connected: `ros2 topic echo /joy`
   - Check if robot is in the right mode
   - Ensure external activation device is connected
   - Verify robot is not in protective stop

4. **Jerky movement**:
   - Adjust deadzone parameter
   - Check network latency
   - Ensure realtime PC has proper RT kernel

### Debugging Commands:

```bash
# Check if joystick is connected and publishing
ros2 topic echo /joy

# Test network connectivity to realtime PC
ping 192.168.18.1

# Check if UDP port is reachable (install netcat if needed)
nc -u 192.168.18.1 8888

# List ROS2 nodes to verify everything is running
ros2 node list

# Check ROS2 topics
ros2 topic list
```

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

## Safety Notes

⚠️ **Important Safety Considerations**:
- Always have the external activation device ready
- Test in a safe environment first
- Be aware of the robot's workspace limits
- Emergency stop button (B) should always be easily accessible
- Start with slow movements and increase speed gradually

## Network Protocol

The system uses a simple UDP protocol with space-separated values:
```
"linear_x linear_y linear_z angular_x angular_y angular_z emergency_stop reset_pose"
```

Example: `"0.1 -0.05 0.0 0.0 0.0 0.2 0 0"`