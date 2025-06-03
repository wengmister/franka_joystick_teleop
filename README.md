# Franka Robot Joystick Control Setup

This system allows you to control a Franka robot using a joystick through ROS2 on your local machine, communicating with libfranka running on a realtime PC.

## Architecture

```
Local Machine (ROS2)           Network (UDP)         Realtime PC (libfranka)
┌─────────────────┐                                  ┌──────────────────────┐
│   Joystick      │            ┌─────────┐           │  franka_joystick_    │
│      ↓          │            │ Command │           │  control             │
│ ROS2 joy_node   │ ────────── │ UDP     │ ────────→ │         ↓            │
│      ↓          │            │ Socket  │           │  Franka Robot        │
│ Joystick        │            └─────────┘           │  (libfranka)         │
│ Publisher       │                                  └──────────────────────┘
└─────────────────┘
```

## Setup Instructions

### 1. Realtime PC Setup

1. **Install libfranka** on your realtime PC (usually Ubuntu with RT kernel):
   ```bash
   sudo apt install libfranka-dev
   ```

2. **Compile the Franka controller**:
   ```bash
   g++ -std=c++17 franka_joystick_control.cpp -lfranka -lpthread -o franka_joystick_control
   ```

3. **Set network permissions** (if needed):
   ```bash
   sudo setcap cap_net_bind_service+ep ./franka_joystick_control
   ```

### 2. Local Machine Setup (ROS2)

1. **Install ROS2** (Humble recommended) and joy package:
   ```bash
   sudo apt install ros-humble-joy
   ```

2. **Create ROS2 workspace**:
   ```bash
   mkdir -p ~/franka_ws/src
   cd ~/franka_ws/src
   ```

3. **Create the package structure**:
   ```bash
   mkdir -p franka_joystick_control/{src,scripts,launch}
   
   # Copy the files:
   # - CMakeLists.txt and package.xml to franka_joystick_control/
   # - franka_joystick_publisher.cpp to franka_joystick_control/src/
   # - franka_joystick_publisher.py to franka_joystick_control/scripts/
   # - joystick_control.launch.py to franka_joystick_control/launch/
   ```

4. **Make Python script executable**:
   ```bash
   chmod +x ~/franka_ws/src/franka_joystick_control/scripts/franka_joystick_publisher.py
   ```

5. **Build the package**:
   ```bash
   cd ~/franka_ws
   colcon build --packages-select franka_joystick_control
   source install/setup.bash
   ```

### 3. Network Configuration

1. **Find IP addresses**:
   - Realtime PC: `ip addr show`
   - Local machine: `ip addr show`

2. **Test connectivity**:
   ```bash
   ping <realtime_pc_ip>
   ```

3. **Configure firewall** (on realtime PC if needed):
   ```bash
   sudo ufw allow 8888/udp
   ```

## Usage

### 1. Start the Franka Controller (Realtime PC)

```bash
# SSH into your realtime PC
ssh user@<realtime_pc_ip>

# Run the controller (replace with your robot's IP)
./franka_joystick_control <robot_ip>
```

Example:
```bash
./franka_joystick_control 172.16.0.2
```

### 2. Start ROS2 Joystick Control (Local Machine)

**Option A: Using Python version (recommended)**
```bash
# Source ROS2
source /opt/ros/humble/setup.bash
source ~/franka_ws/install/setup.bash

# Launch with your realtime PC IP
ros2 launch franka_joystick_control joystick_control.launch.py robot_pc_ip:=<realtime_pc_ip>
```

**Option B: Using C++ version**
```bash
ros2 launch franka_joystick_control joystick_control.launch.py robot_pc_ip:=<realtime_pc_ip> use_python:=false
```

**Option C: Manual startup**
```bash
# Terminal 1: Joy node
ros2 run joy joy_node

# Terminal 2: Joystick publisher
ros2 run franka_joystick_control franka_joystick_publisher.py --ros-args -p robot_pc_ip:=<realtime_pc_ip>
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
# Check joystick input
ros2 topic echo /joy

# Check network connectivity
nc -u <realtime_pc_ip> 8888

# Monitor robot state (if you have franka_ros2)
ros2 topic echo /franka_robot_state_broadcaster/robot_state
```

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