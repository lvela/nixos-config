# ROS 2 TurtleBot 4 Setup on NixOS - Complete Guide

## 🎉 Setup Status: COMPLETE ✅

Your ROS 2 Jazzy installation is now fully functional and ready for TurtleBot 4 development!

## 📋 System Configuration

| Component | Configuration |
|-----------|---------------|
| **Operating System** | NixOS 25.05 with Flakes |
| **ROS Version** | ROS 2 Jazzy |
| **Host System IP** | 192.168.1.38 (Beelink) |
| **TurtleBot 4 IP** | 192.168.1.70 |
| **ROS Domain ID** | 0 |
| **RMW Implementation** | rmw_fastrtps_cpp |
| **Python Version** | 3.12 |

## 🔧 What Was Fixed

### Original Issues Resolved:
1. **Missing Python Dependencies**
   - ❌ `No module named 'yaml'` → ✅ Added `python3Packages.pyyaml`
   - ❌ `No module named 'numpy'` → ✅ Added `python3Packages.numpy`
   - ❌ `No module named 'rosidl_parser'` → ✅ Added `rosPackages.jazzy.rosidl-parser`
   - ❌ `No module named 'rpyutils'` → ✅ Added `rosPackages.jazzy.rpyutils`
   - ❌ `No module named 'ament_index_python'` → ✅ Added `rosPackages.jazzy.ament-index-python`

2. **RMW Library Issues**
   - ❌ `librmw_fastrtps_cpp.so: cannot open shared object file` → ✅ Added complete FastRTPS packages
   - ❌ `librmw_dds_common__rosidl_typesupport_fastrtps_cpp.so` missing → ✅ Added `rmw-dds-common`

3. **Environment Configuration**
   - ✅ Fixed `PYTHONPATH`, `LD_LIBRARY_PATH`, `CMAKE_PREFIX_PATH`
   - ✅ Set proper ROS environment variables
   - ✅ Configured firewall for ROS multicast traffic

## 📦 Installed ROS Packages

### Core ROS Packages
- `ros-core`, `ros-base`, `desktop`
- `ros2cli`, `ros2launch`, `ros2run`, `ros2node`, `ros2topic`, `ros2service`, `ros2param`

### Python Bindings
- `rclpy`, `rclcpp`
- `rosidl-runtime-py`, `rpyutils`, `ament-index-python`

### Message Types
- `std-msgs`, `geometry-msgs`, `sensor-msgs`, `nav-msgs`
- `rcl-interfaces`, `builtin-interfaces`
- `turtlebot4-msgs`, `nav2-msgs`

### ROSIDL Support
- `rosidl-default-runtime`, `rosidl-default-generators`
- `rosidl-parser`, `rosidl-adapter`
- Complete typesupport packages for C, C++, and FastRTPS

### RMW Implementation
- `rmw`, `rmw-implementation`, `rmw-fastrtps-cpp`
- `rmw-dds-common`, `fastrtps`, `fastcdr`

### Tools
- `teleop-twist-keyboard`

## ✅ Current Working Status

### Environment Variables
```bash
ROS_DISTRO=jazzy
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
AMENT_PREFIX_PATH=/run/current-system/sw
ROS_VERSION=2
```

### Python Imports
All ROS Python modules import successfully:
- ✅ `rclpy`
- ✅ `std_msgs.msg`
- ✅ `geometry_msgs.msg`
- ✅ `rosidl_parser`
- ✅ `rosidl_adapter`

### CLI Tools
- ✅ `ros2 --help`
- ✅ `ros2 node list`
- ✅ `ros2 topic list`

### Network Connectivity
- ✅ TurtleBot 4 ping successful (192.168.1.70)
- ✅ SSH port 22 open
- ✅ Web interface port 8080 open

### ROS Discovery
- ✅ Local ROS nodes detected
- ✅ ROS topics visible (`/parameter_events`, `/rosout`)

## 🚀 Quick Start Guide

### 1. Connect to TurtleBot
```bash
ssh ubuntu@192.168.1.70
# Default password: turtlebot4
```

### 2. Start TurtleBot ROS System
On the TurtleBot:
```bash
ros2 launch turtlebot4_bringup standard.launch.py
```

### 3. Test Discovery
Back on your host system:
```bash
ros2 node list
ros2 topic list
```

### 4. Control the Robot

#### Keyboard Control
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### Direct Commands
```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.5}, angular: {z: 0.0}}'

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.5}}'
```

## 🌐 Web Interface

Access the TurtleBot 4 web interface at: **http://192.168.1.70:8080**

## 📁 Configuration Files

### Main NixOS Configuration
- **File**: `hosts/beelink.nix`
- **Flake**: `flake.nix` (uses `nix-ros-overlay`)
- **Hardware**: `hardware/beelink-hardware.nix`

### Key Configuration Sections
```nix
# ROS environment setup
environment.sessionVariables = {
  ROS_DOMAIN_ID = "0";
  RMW_IMPLEMENTATION = "rmw_fastrtps_cpp";
};

# Firewall configuration for ROS
networking.firewall = {
  allowedTCPPorts = [ 22 8080 11311 ];
  allowedUDPPorts = [ 7400 7401 7402 7403 ];
  allowedUDPPortRanges = [
    { from = 7400; to = 7499; }  # FastRTPS discovery
  ];
};
```

## 🔍 Troubleshooting

### Common Issues and Solutions

1. **ROS Discovery Fails**
   - Check `ROS_DOMAIN_ID` matches between systems
   - Verify firewall allows UDP ports 7400-7499
   - Ensure both systems are on the same network

2. **Import Errors**
   - All dependencies are now installed
   - If issues persist, rebuild: `sudo nixos-rebuild switch --flake .`

3. **Connection Issues**
   - Verify TurtleBot is powered on and connected to WiFi
   - Check IP addresses with `ping 192.168.1.70`
   - Ensure TurtleBot ROS system is running

4. **Command Timeouts**
   - `ros2 topic pub` waiting for subscribers is normal when TurtleBot ROS isn't running
   - Start TurtleBot ROS system first

### Useful Commands

```bash
# Check ROS environment
env | grep -E "(ROS|AMENT|RMW)" | sort

# Test ROS installation
python3 test_ros_setup.py

# Check network connectivity
python3 test_turtlebot.py

# Monitor topics
ros2 topic echo /cmd_vel
ros2 topic echo /odom

# List all available topics
ros2 topic list -t

# Get topic info
ros2 topic info /cmd_vel
```

## 🎯 Next Steps for Development

1. **Sensor Integration**
   - Camera feeds: `/camera/image_raw`
   - Lidar data: `/scan`
   - Odometry: `/odom`

2. **Navigation Setup**
   - SLAM mapping
   - Nav2 navigation stack
   - Waypoint navigation

3. **Custom Node Development**
   - Create Python/C++ ROS nodes
   - Implement custom behaviors
   - Sensor data processing

4. **Advanced Features**
   - Multi-robot coordination
   - Computer vision integration
   - Machine learning applications

## 📚 Additional Resources

- [TurtleBot 4 Documentation](https://turtlebot.github.io/turtlebot4-user-manual/)
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [NixOS ROS Overlay](https://github.com/lopsided98/nix-ros-overlay)

---

**Setup completed on**: July 20, 2025  
**Total setup time**: Multiple iterations to resolve all dependencies  
**Status**: ✅ Ready for robotics development!

🤖 **Happy robot programming!**
