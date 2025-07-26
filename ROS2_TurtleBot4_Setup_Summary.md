# ROS 2 TurtleBot 4 Setup on NixOS - Complete Guide

## 🎉 Setup Status: COMPLETE ✅

Your ROS 2 Jazzy installation is fully functional with cross-system communication to TurtleBot 4!

## 📋 System Configuration

| Component | Configuration |
|-----------|---------------|
| **Operating System** | NixOS 25.05 with Flakes |
| **ROS Version** | ROS 2 Jazzy |
| **Host System IP** | 192.168.1.38 (Beelink) |
| **TurtleBot 4 IP** | 192.168.1.70 |
| **ROS Domain ID** | 0 |
| **RMW Implementation** | rmw_fastrtps_cpp |
| **Discovery Mode** | Discovery Server (TurtleBot: 11811) |
| **Python Version** | 3.12 |

## 🔧 Key Components Working

### Core Functionality
- ✅ **ROS 2 Jazzy** - Full installation with all dependencies
- ✅ **Cross-system Discovery** - Using TurtleBot's Discovery Server (192.168.1.70:11811)
- ✅ **SSH Access** - Automated key-based authentication to TurtleBot
- ✅ **Robot Control** - Real-time topic communication (`/cmd_vel`, `/odom`, `/scan`)
- ✅ **Environment Management** - Automated ROS environment setup via NixOS

### Major Issues Resolved
1. **ROS Discovery Configuration** *(Fixed July 26, 2025)*
   - ✅ Identified TurtleBot uses Discovery Server mode (`127.0.0.1:11811`)
   - ✅ Configured host to use TurtleBot's Discovery Server (`192.168.1.70:11811`)
   - ✅ Set `ROS_SUPER_CLIENT=true` for client mode
   - ✅ Cross-system topic discovery now working with all TurtleBot topics visible

2. **ROS 2 Dependencies** *(Resolved)*
   - ✅ All Python dependencies installed (`pyyaml`, `numpy`, `psutil`)
   - ✅ Complete ROS package set with message types and tools
   - ✅ FastRTPS implementation configured
   - ✅ ROS daemon working properly

3. **SSH Integration** *(Automated)*
   - ✅ Automated SSH key generation and deployment
   - ✅ System-wide SSH configuration for TurtleBot access
   - ✅ Helper scripts for easy connection and management

4. **Teleop Keyboard Control** *(Fixed & Tested July 26, 2025)*
   - ✅ Identified TurtleBot 4 uses TwistStamped messages on `/cmd_vel`
   - ✅ Configured teleop_twist_keyboard to use `/cmd_vel_unstamped` topic
   - ✅ Created `turtlebot-teleop` helper script for easy control
   - ✅ Added troubleshooting script for teleop issues
   - ✅ **CONFIRMED WORKING**: Robot successfully responds to both direct commands and teleop keyboard control

## ✅ Current Working Status

### Core ROS Environment
```bash
ROS_DISTRO=jazzy
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_DISCOVERY_SERVER=192.168.1.70:11811
ROS_SUPER_CLIENT=true
ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```

### Available TurtleBot Topics
- ✅ **Navigation**: `/cmd_vel`, `/odom`, `/tf`, `/tf_static`
- ✅ **Sensors**: `/scan` (LiDAR), `/imu`, `/battery_state`
- ✅ **Camera**: `/oakd/rgb/preview/image_raw`, `/oakd/rgb/preview/camera_info`
- ✅ **Robot State**: `/joint_states`, `/robot_description`, `/diagnostics`
- ✅ **Control**: `/joy`, `/hmi/buttons`, `/hmi/led`

### System Status
- ✅ ROS daemon running successfully
- ✅ Cross-system discovery working
- ✅ SSH key authentication configured
- ✅ Real-time data streaming from TurtleBot
- ✅ Robot control commands working
- ✅ **Teleop keyboard control CONFIRMED WORKING** - Robot moves with direct commands and keyboard input

## 🚀 Quick Start Guide

### 1. Connect to TurtleBot
```bash
# Using SSH alias (password-free)
ssh turtlebot

# Or using helper script
turtlebot-connect
```

### 2. Verify ROS Communication
```bash
# Check all available topics
ros2 topic list

# Monitor robot odometry
ros2 topic echo /odom

# Check battery status
ros2 topic echo /battery_state
```

### 3. Control the Robot

#### Keyboard Control (Fixed & Working!)
```bash
# Option 1: Use the new helper script (after rebuilding NixOS)
turtlebot-teleop

# Option 2: Manual command with proper topic remapping
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel_unstamped

# Option 3: Use troubleshooting script if issues occur
./turtlebot-teleop-fix.sh
```

#### Direct Commands (TurtleBot uses TwistStamped)
```bash
# Move forward
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: "base_link"}, twist: {linear: {x: 0.2}}}'

# Turn left
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: "base_link"}, twist: {angular: {z: 0.5}}}'

# Stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: "base_link"}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}'
```

### 4. Access Web Interface
Open: **http://192.168.1.70:8080**

## � Configuration Details

### NixOS Configuration
- **File**: `hosts/beelink.nix`
- **Flake**: `flake.nix` (uses `nix-ros-overlay`)

### Key Settings
```nix
# ROS environment
environment.sessionVariables = {
  ROS_DOMAIN_ID = "0";
  RMW_IMPLEMENTATION = "rmw_fastrtps_cpp";
  ROS_AUTOMATIC_DISCOVERY_RANGE = "SUBNET";
};

# Discovery Server configuration
programs.bash.interactiveShellInit = ''
  export ROS_DISCOVERY_SERVER="192.168.1.70:11811"
  export ROS_SUPER_CLIENT="true"
'';

# Firewall for ROS communication
networking.firewall = {
  allowedTCPPorts = [ 22 8080 ];  # SSH, TurtleBot web interface
  allowedUDPPorts = [ 7400 7401 7402 7403 11811 ];  # FastRTPS + Discovery Server
  allowedUDPPortRanges = [{ from = 7400; to = 7499; }];
};
```

### SSH Configuration
```bash
# System-wide SSH config (/etc/ssh/ssh_config)
Host turtlebot
    HostName 192.168.1.70
    User ubuntu
    IdentityFile ~/.ssh/id_ed25519_turtlebot
    IdentitiesOnly yes
```

## 🔍 Troubleshooting

### Common Issues

1. **No TurtleBot Topics Visible**
   ```bash
   # Check Discovery Server connection
   export ROS_DISCOVERY_SERVER="192.168.1.70:11811"
   export ROS_SUPER_CLIENT="true"
   ros2 daemon stop && ros2 daemon start
   ```

2. **SSH Connection Issues**
   ```bash
   # Test SSH connectivity
   turtlebot-ssh-setup test
   
   # Regenerate keys if needed
   turtlebot-ssh-setup setup
   ```

3. **Environment Issues**
   ```bash
   # Rebuild NixOS configuration
   sudo nixos-rebuild switch --flake .
   
   # Check ROS environment
   env | grep -E "(ROS|RMW)" | sort
   ```

4. **Teleop Keyboard Not Working**
   ```bash
   # Use the correct topic remapping
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel_unstamped
   
   # Or use the helper script
   turtlebot-teleop
   
   # Or run the troubleshooting script
   ./turtlebot-teleop-fix.sh
   ```

### Useful Commands
```bash
# Monitor all topics
ros2 topic list -t

# Check topic details
ros2 topic info /cmd_vel

# Remote robot status
ssh turtlebot 'systemctl status turtlebot4.service'

# Real-time odometry
ros2 topic echo /odom

# Teleop with proper remapping (FIXED!)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel_unstamped

# Quick teleop helper
turtlebot-teleop

# Teleop troubleshooting
./turtlebot-teleop-fix.sh
```

## 🎯 Development Examples

### TurtleBot 4 Teleop Solutions
```bash
# Method 1: Helper script (available after rebuilding NixOS)
turtlebot-teleop

# Method 2: Manual remapping (works immediately)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel_unstamped

# Method 3: Troubleshooting script (comprehensive solution)
./turtlebot-teleop-fix.sh

# Method 4: Direct TwistStamped commands
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: "base_link"}, twist: {linear: {x: 0.2}}}'
```

### Basic Robot Control
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
    def move_forward(self, speed=0.2):
        msg = TwistStamped()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = speed
        self.publisher.publish(msg)

# Usage: ros2 run your_package turtlebot_controller
```

### Sensor Data Monitoring
```bash
# Monitor LiDAR data
ros2 topic echo /scan

# Check robot odometry
ros2 topic echo /odom

# Monitor battery status
ros2 topic echo /battery_state

# Camera feed
ros2 topic echo /oakd/rgb/preview/image_raw
```

### Next Development Steps
1. **SLAM Mapping** ✅ - Create maps using SLAM toolbox (setup complete!)
2. **Autonomous Navigation** - Implement waypoint navigation with Nav2
3. **Computer Vision** - Process camera feeds for object detection
4. **Custom Behaviors** - Develop application-specific robot behaviors

## 🗺️ SLAM Mapping Guide - **READY FOR MAPPING!**

### ✅ SLAM Status: Ready to Map
Your system includes all necessary SLAM components! While RViz2 has graphics issues in NixOS, you can create maps using command-line tools and alternative visualization.

### **Quick Start - Command Line SLAM (Recommended)**
```bash
# Terminal 1: Launch SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py

# Terminal 2: Control robot for mapping
turtlebot-teleop
# OR: Manual commands
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: "base_link"}, twist: {linear: {x: 0.2}}}'

# Terminal 3: Monitor mapping progress
ros2 topic echo /map --once  # Check if map is being created
ros2 topic list | grep map   # See map-related topics

# Save map when complete
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: 'my_house_map'}}"
```

### **Alternative Visualization Options**

#### 1. **RQt Graph Viewer** (Works in NixOS)
```bash
# View ROS topic graph and connections
rqt_graph

# Monitor topics with plots
rqt_plot /scan/ranges[360]  # Monitor lidar data
```

#### 2. **Web-based Visualization**
```bash
# Use TurtleBot's built-in web interface
# Open: http://192.168.1.70:8080
# This includes robot status, camera feed, and basic mapping
```

#### 3. **Command Line Monitoring**
```bash
# Monitor real-time SLAM data
ros2 topic echo /map_metadata    # Map info
ros2 topic echo /scan           # LiDAR data
ros2 topic hz /map              # Map update rate
ros2 node list | grep slam      # SLAM nodes status
```

### **RViz2 Solution: Docker-based Visualization** 
**✅ RECOMMENDED APPROACH**

RViz2 has OGRE library hardcoded path issues in NixOS. The solution is to run RViz2 in Docker:

```bash
# Setup Docker RViz2 (one-time setup)
./docker-rviz2-setup.sh

# Option 1: Complete SLAM with Docker RViz2
./docker-slam-complete.sh

# Option 2: Just Docker RViz2 (if SLAM already running)
./run-docker-rviz2.sh

# Option 3: Simple Docker RViz2 command
./rviz2-docker
```

**Benefits**:
- ✅ Bypasses OGRE hardcoded path issues
- ✅ Full graphics acceleration with native performance
- ✅ Runs locally on Beelink (no remote X11 needed)
- ✅ Same network as host (ROS discovery works perfectly)
- ✅ Native ROS Jazzy environment in container

### **Mapping Workflow - Docker RViz2** 
1. **Setup Docker RViz2** (one-time): `./docker-rviz2-setup.sh`
2. **Complete SLAM Session**: `./docker-slam-complete.sh`
   - OR manually:
     - Terminal 1: `ros2 launch slam_toolbox online_async_launch.py`
     - Terminal 2: `./run-docker-rviz2.sh`
3. **Move Robot**: Use `turtlebot-teleop` or manual commands
4. **Monitor Progress**: RViz2 shows real-time mapping
5. **Save Map**: `ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_house_map'}}"`

### **Mapping Tips**
- **Move slowly** through your house
- **Cover all areas** systematically
- **Return to starting point** for loop closure
- **Monitor topics** to ensure SLAM is working
- **Save multiple versions** of your map

Your SLAM setup is complete and functional - you can create maps without RViz2!

## 🎮 Teleop Keyboard Control - SOLVED!

### The Problem
TurtleBot 4 uses **TwistStamped messages** on `/cmd_vel`, but `teleop_twist_keyboard` publishes standard **Twist messages**. The TurtleBot has a `create3_republisher` that handles this conversion, but it requires the correct topic remapping.

### The Solution
The TurtleBot's `create3_republisher` remaps:
- `/cmd_vel_unstamped` (Twist) → robot control  
- `/cmd_vel` (TwistStamped) → robot control

### Working Methods

#### 1. Quick Fix (Use Right Now)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel_unstamped
```

#### 2. Helper Script (After Rebuilding NixOS)
```bash
# Rebuild first to get the new command
sudo nixos-rebuild switch --flake .

# Then use the helper
turtlebot-teleop
```

#### 3. Troubleshooting Script
```bash
# Comprehensive solution with diagnostics
./turtlebot-teleop-fix.sh
```

#### 4. Manual TwistStamped Commands
```bash
# Move forward
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: "base_link"}, twist: {linear: {x: 0.2}}}'

# Turn left  
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: "base_link"}, twist: {angular: {z: 0.5}}}'

# Stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped \
  '{header: {frame_id: "base_link"}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}'
```

### Keyboard Controls
Once teleop is running:
- **i/j/k/l** - Move forward/left/right/backward  
- **u/o/m/,** - Diagonal movements
- **q/z** - Increase/decrease max speeds
- **w/x** - Increase/decrease linear speed only
- **e/c** - Increase/decrease angular speed only  
- **Any other key** - STOP
- **Ctrl+C** - Quit

### Common Issues & Fixes

**Topics Not Visible:**
```bash
ros2 daemon stop && ros2 daemon start
ros2 topic list | grep cmd_vel
```

**TurtleBot Disconnected:**
```bash
ping 192.168.1.70
ssh turtlebot 'systemctl status turtlebot4.service'
```

**Discovery Issues:**
```bash
export ROS_DISCOVERY_SERVER="192.168.1.70:11811"
export ROS_SUPER_CLIENT="true"
ros2 daemon stop && ros2 daemon start
```

## 📚 Resources
- [TurtleBot 4 Documentation](https://turtlebot.github.io/turtlebot4-user-manual/)
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [NixOS ROS Overlay](https://github.com/lopsided98/nix-ros-overlay)

---

**Status**: ✅ **Ready for robotics development with full ROS 2 functionality!**  
**Last Updated**: July 26, 2025  
🤖 **Happy robot programming!**
