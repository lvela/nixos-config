#!/usr/bin/env python3
"""
ROS 2 Installation and TurtleBot Connection Test
Tests both local ROS setup and TurtleBot connectivity
"""

import subprocess
import socket
import sys
import os

def test_ros_installation():
    """Test local ROS 2 installation"""
    print("🔧 ROS 2 Installation Test")
    print("=" * 40)
    
    # Test environment variables
    print("1. Testing environment variables...")
    ros_vars = ['ROS_DISTRO', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'AMENT_PREFIX_PATH']
    for var in ros_vars:
        value = os.environ.get(var, 'NOT SET')
        status = "✅" if value != 'NOT SET' else "❌"
        print(f"   {status} {var}: {value}")
    
    # Test Python imports
    print("\n2. Testing Python ROS imports...")
    imports_to_test = [
        'rclpy',
        'std_msgs.msg',
        'geometry_msgs.msg',
        'rosidl_parser',
        'rosidl_adapter',
    ]
    
    for module in imports_to_test:
        try:
            __import__(module)
            print(f"   ✅ {module}: OK")
        except ImportError as e:
            print(f"   ❌ {module}: FAILED - {e}")
        except Exception as e:
            print(f"   ⚠️  {module}: ERROR - {e}")
    
    # Test ROS CLI tools
    print("\n3. Testing ROS CLI tools...")
    cli_commands = [
        ['ros2', '--help'],
        ['ros2', 'node', 'list'],
        ['ros2', 'topic', 'list'],
    ]
    
    for cmd in cli_commands:
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            if result.returncode == 0:
                print(f"   ✅ {' '.join(cmd[:2])}: OK")
            else:
                print(f"   ❌ {' '.join(cmd[:2])}: FAILED")
                if result.stderr:
                    print(f"      Error: {result.stderr[:100]}...")
        except subprocess.TimeoutExpired:
            print(f"   ⏱️  {' '.join(cmd[:2])}: TIMEOUT")
        except Exception as e:
            print(f"   ❌ {' '.join(cmd[:2])}: ERROR - {e}")

def test_turtlebot_connection():
    """Test TurtleBot connectivity"""
    print("\n🤖 TurtleBot 4 Connection Test")
    print("=" * 40)
    
    turtlebot_ip = "192.168.1.70"
    
    # Test ping
    print("1. Testing ping connectivity...")
    try:
        result = subprocess.run(['ping', '-c', '3', turtlebot_ip], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print("   ✅ Ping successful!")
        else:
            print("   ❌ Ping failed!")
    except Exception as e:
        print(f"   ❌ Ping error: {e}")
    
    # Test ports
    print("2. Testing important ports...")
    ports_to_test = [
        (22, "SSH"),
        (8080, "Web Interface"),
        (7400, "FastRTPS Discovery"),
        (11311, "ROS Master (if ROS1 bridge)"),
    ]
    
    for port, description in ports_to_test:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3)
            result = sock.connect_ex((turtlebot_ip, port))
            sock.close()
            if result == 0:
                print(f"   ✅ Port {port} ({description}): Open")
            else:
                print(f"   ⚪ Port {port} ({description}): Closed")
        except Exception as e:
            print(f"   ❌ Port {port} ({description}): Error - {e}")

def test_ros_turtlebot_discovery():
    """Test ROS discovery with TurtleBot"""
    print("\n🔍 ROS Discovery Test")
    print("=" * 40)
    
    print("1. Attempting to discover ROS nodes...")
    try:
        # Set the same domain ID as TurtleBot
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = '0'
        
        result = subprocess.run(['ros2', 'node', 'list'], 
                              capture_output=True, text=True, timeout=15, env=env)
        if result.returncode == 0:
            nodes = result.stdout.strip().split('\n')
            print(f"   ✅ Found {len(nodes)} ROS nodes:")
            for node in nodes[:5]:  # Show first 5 nodes
                print(f"      - {node}")
            if len(nodes) > 5:
                print(f"      ... and {len(nodes) - 5} more")
        else:
            print("   ❌ No nodes discovered")
            if result.stderr:
                print(f"      Error: {result.stderr}")
    except subprocess.TimeoutExpired:
        print("   ⏱️  Discovery timeout (this is normal if TurtleBot is not running)")
    except Exception as e:
        print(f"   ❌ Discovery error: {e}")
    
    print("\n2. Attempting to list ROS topics...")
    try:
        result = subprocess.run(['ros2', 'topic', 'list'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            topics = result.stdout.strip().split('\n')
            print(f"   ✅ Found {len(topics)} ROS topics:")
            for topic in topics[:5]:  # Show first 5 topics
                print(f"      - {topic}")
            if len(topics) > 5:
                print(f"      ... and {len(topics) - 5} more")
        else:
            print("   ❌ No topics discovered")
    except Exception as e:
        print(f"   ❌ Topic discovery error: {e}")

def suggest_next_steps():
    """Suggest next steps based on test results"""
    print("\n🔧 Next Steps")
    print("=" * 40)
    print("If ROS tests passed:")
    print("  1. Try: ros2 run teleop_twist_keyboard teleop_twist_keyboard")
    print("  2. SSH to TurtleBot: ssh ubuntu@192.168.1.70")
    print("  3. On TurtleBot, run: ros2 launch turtlebot4_bringup standard.launch.py")
    print()
    print("If ROS tests failed:")
    print("  1. Rebuild NixOS: sudo nixos-rebuild switch --flake .")
    print("  2. Restart your shell or reboot")
    print("  3. Check firewall settings")
    print()
    print("TurtleBot Web Interface:")
    print("  http://192.168.1.70:8080")
    print()
    print("Manual testing commands:")
    print("  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist")
    print("  ros2 topic echo /odom")

if __name__ == "__main__":
    try:
        test_ros_installation()
        test_turtlebot_connection()
        test_ros_turtlebot_discovery()
        suggest_next_steps()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(1)
