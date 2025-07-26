#!/usr/bin/env python3
"""
Simple TurtleBot network test script
This bypasses local ROS setup issues by using direct network commands
"""

import subprocess
import socket
import time

def test_turtlebot_connection():
    """Test basic connectivity to TurtleBot"""
    turtlebot_ip = "192.168.1.70"
    
    print("🤖 TurtleBot 4 Connection Test")
    print("=" * 40)
    
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
    
    # Test SSH port
    print("2. Testing SSH port (22)...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        result = sock.connect_ex((turtlebot_ip, 22))
        sock.close()
        if result == 0:
            print("   ✅ SSH port is open!")
        else:
            print("   ❌ SSH port is closed!")
    except Exception as e:
        print(f"   ❌ SSH test error: {e}")
    
    # Test common ROS ports
    print("3. Testing common ROS ports...")
    ros_ports = [11311, 7400, 7401, 7402]  # Common ROS/DDS ports
    for port in ros_ports:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2)
            result = sock.connect_ex((turtlebot_ip, port))
            sock.close()
            if result == 0:
                print(f"   ✅ Port {port} is open!")
            else:
                print(f"   ⚪ Port {port} is closed/filtered")
        except Exception as e:
            print(f"   ❌ Port {port} test error: {e}")

def suggest_next_steps():
    """Suggest next steps for TurtleBot testing"""
    print("\n🔧 Suggested Next Steps:")
    print("=" * 40)
    print("1. Access TurtleBot web interface:")
    print("   http://192.168.1.70:8080")
    print("   (May require different port, try 8000, 8080, 80)")
    print()
    print("2. SSH into TurtleBot for direct testing:")
    print("   ssh ubuntu@192.168.1.70")
    print("   Default password: turtlebot4")
    print()
    print("3. Manual ROS commands to try on TurtleBot:")
    print("   ros2 node list")
    print("   ros2 topic list")
    print("   ros2 topic echo /odom")
    print()
    print("4. Camera feed test:")
    print("   ros2 topic echo /image_raw")

if __name__ == "__main__":
    test_turtlebot_connection()
    suggest_next_steps()
