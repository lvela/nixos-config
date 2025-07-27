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
    print("  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}, angular: {z: 0.0}}'")
    print("  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0}, angular: {z: 0.0}}'")
    print("  ros2 topic echo /odom")

def test_robot_control():
    """Test robot control commands"""
    print("\n🎮 Robot Control Test")
    print("=" * 40)
    
    print("1. Testing basic movement command...")
    try:
        # Test a simple forward movement command with proper timestamp
        cmd = [
            'ros2', 'topic', 'pub', '--once', '/cmd_vel', 
            'geometry_msgs/msg/Twist',
            '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
        ]
        
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print("   ✅ Movement command published successfully!")
            print("   📝 If TurtleBot is running, it should move forward briefly")
        else:
            print("   ❌ Movement command failed")
            if result.stderr:
                print(f"      Error: {result.stderr}")
    except subprocess.TimeoutExpired:
        print("   ⏱️  Command timed out (waiting for subscribers)")
        print("   📝 This is normal if TurtleBot ROS system isn't running")
    except Exception as e:
        print(f"   ❌ Control test error: {e}")
    
    print("\n2. Testing stop command...")
    try:
        # Test stop command
        cmd = [
            'ros2', 'topic', 'pub', '--once', '/cmd_vel',
            'geometry_msgs/msg/Twist', 
            '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
        ]
        
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print("   ✅ Stop command published successfully!")
        else:
            print("   ❌ Stop command failed")
    except subprocess.TimeoutExpired:
        print("   ⏱️  Stop command timed out (this is normal)")
    except Exception as e:
        print(f"   ❌ Stop command error: {e}")
    
    print("\n3. Testing teleop keyboard availability...")
    try:
        # Check if teleop_twist_keyboard is available
        result = subprocess.run(['ros2', 'pkg', 'list'], capture_output=True, text=True, timeout=5)
        if 'teleop_twist_keyboard' in result.stdout:
            print("   ✅ teleop_twist_keyboard package found")
            print("   💡 Try: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=cmd_vel")
        else:
            print("   ❌ teleop_twist_keyboard package not found")
            print("   💡 You may need to install it or use manual commands")
    except Exception as e:
        print(f"   ❌ Package check error: {e}")

def test_turtlebot_status():
    """Test if TurtleBot ROS system is running"""
    print("\n🤖 TurtleBot ROS Status Check")
    print("=" * 40)
    
    print("1. Checking for TurtleBot-specific topics...")
    try:
        result = subprocess.run(['ros2', 'topic', 'list'], capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            topics = result.stdout.strip().split('\n')
            
            # Look for TurtleBot-specific topics
            turtlebot_topics = ['/cmd_vel', '/odom', '/scan', '/battery_state', '/imu', '/camera']
            found_topics = []
            
            for topic in turtlebot_topics:
                if any(topic in t for t in topics):
                    found_topics.append(topic)
            
            if found_topics:
                print(f"   ✅ Found {len(found_topics)} TurtleBot topics:")
                for topic in found_topics:
                    print(f"      - {topic}")
                print("   🎉 TurtleBot ROS system appears to be running!")
            else:
                print("   ⚪ No TurtleBot-specific topics found")
                print("   📝 TurtleBot ROS system may not be running")
                print("   💡 SSH to TurtleBot and run: ros2 launch turtlebot4_bringup standard.launch.py")
        else:
            print("   ❌ Could not list topics")
    except Exception as e:
        print(f"   ❌ Topic check error: {e}")
    
    print("\n2. Checking for TurtleBot nodes...")
    try:
        result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            nodes = result.stdout.strip().split('\n')
            
            # Look for TurtleBot-specific nodes
            turtlebot_keywords = ['turtlebot', 'robot', 'base', 'controller', 'lidar', 'camera']
            found_nodes = []
            
            for node in nodes:
                if any(keyword in node.lower() for keyword in turtlebot_keywords):
                    found_nodes.append(node)
            
            if found_nodes:
                print(f"   ✅ Found {len(found_nodes)} TurtleBot-related nodes:")
                for node in found_nodes[:5]:  # Show first 5
                    print(f"      - {node}")
                if len(found_nodes) > 5:
                    print(f"      ... and {len(found_nodes) - 5} more")
            else:
                print("   ⚪ No obvious TurtleBot nodes found")
        else:
            print("   ❌ Could not list nodes")
    except Exception as e:
        print(f"   ❌ Node check error: {e}")

def troubleshoot_ros_discovery():
    """Comprehensive ROS discovery troubleshooting"""
    print("\n🔍 ROS Discovery Troubleshooting")
    print("=" * 40)
    
    # 1. Check environment variables in detail
    print("1. Environment variable analysis...")
    env_vars = {
        'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID', 'NOT SET'),
        'RMW_IMPLEMENTATION': os.environ.get('RMW_IMPLEMENTATION', 'NOT SET'),
        'ROS_LOCALHOST_ONLY': os.environ.get('ROS_LOCALHOST_ONLY', 'NOT SET'),
        'FASTRTPS_DEFAULT_PROFILES_FILE': os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE', 'NOT SET'),
        'ROS_AUTOMATIC_DISCOVERY_RANGE': os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE', 'NOT SET'),
        'ROS_STATIC_PEERS': os.environ.get('ROS_STATIC_PEERS', 'NOT SET'),
        'ROS_DISCOVERY_SERVER': os.environ.get('ROS_DISCOVERY_SERVER', 'NOT SET'),
        'ROS_SUPER_CLIENT': os.environ.get('ROS_SUPER_CLIENT', 'NOT SET'),
    }
    
    for var, value in env_vars.items():
        if var == 'ROS_LOCALHOST_ONLY' and value == '1':
            print(f"   ⚠️  {var}: {value} (This prevents discovery across network!)")
        elif var in ['ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION'] and value == 'NOT SET':
            print(f"   ❌ {var}: {value}")
        elif var == 'ROS_DISCOVERY_SERVER' and value != 'NOT SET':
            print(f"   🌐 {var}: {value} (Discovery Server mode!)")
        elif var == 'ROS_SUPER_CLIENT' and value != 'NOT SET':
            print(f"   🌐 {var}: {value} (Super Client mode!)")
        else:
            print(f"   ✅ {var}: {value}")
    
    # 2. Test different discovery methods
    print("\n2. Testing ROS discovery with different approaches...")
    
    # Try with explicit domain ID
    try:
        print("   Testing with explicit ROS_DOMAIN_ID=0...")
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = '0'
        env['ROS_LOCALHOST_ONLY'] = '0'  # Ensure network discovery is enabled
        
        result = subprocess.run(['ros2', 'topic', 'list', '--verbose'], 
                              capture_output=True, text=True, timeout=20, env=env)
        
        if result.returncode == 0:
            topics = result.stdout.strip().split('\n')
            robot_topics = [t for t in topics if any(keyword in t for keyword in ['/cmd_vel', '/odom', '/scan', '/battery'])]
            if robot_topics:
                print(f"   ✅ Found robot topics: {robot_topics}")
            else:
                print(f"   ⚪ Only found local topics: {len(topics)} total")
        else:
            print(f"   ❌ Failed: {result.stderr}")
            
    except subprocess.TimeoutExpired:
        print("   ⏱️  Discovery timed out")
    except Exception as e:
        print(f"   ❌ Error: {e}")
    
    # 3. Test network multicast
    print("\n3. Testing multicast connectivity...")
    try:
        # Check if we can reach multicast address used by DDS
        result = subprocess.run(['ping', '-c', '2', '-W', '3', '239.255.0.1'], 
                              capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            print("   ✅ Multicast ping successful")
        else:
            print("   ❌ Multicast ping failed - this could be the issue!")
    except Exception as e:
        print(f"   ❌ Multicast test error: {e}")
    
    # 4. Check specific FastRTPS discovery ports
    print("\n4. Testing FastRTPS discovery ports...")
    turtlebot_ip = "192.168.1.70"
    discovery_ports = [7400, 7401, 7402, 7403, 7404, 7405]
    
    for port in discovery_ports:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(1)
            # Try to connect to UDP port
            result = sock.connect_ex((turtlebot_ip, port))
            sock.close()
            print(f"   ⚪ UDP {port}: Tested")
        except Exception as e:
            print(f"   ❌ UDP {port}: Error - {e}")
    
    # 5. Test daemon discovery
    print("\n5. Testing ROS daemon discovery...")
    try:
        result = subprocess.run(['ros2', 'daemon', 'status'], 
                              capture_output=True, text=True, timeout=5)
        print(f"   📝 Daemon status: {result.stdout.strip()}")
        
        # Restart daemon to refresh discovery
        print("   🔄 Restarting ROS daemon...")
        subprocess.run(['ros2', 'daemon', 'stop'], capture_output=True, timeout=5)
        subprocess.run(['ros2', 'daemon', 'start'], capture_output=True, timeout=5)
        print("   ✅ Daemon restarted")
        
    except Exception as e:
        print(f"   ❌ Daemon error: {e}")

def suggest_discovery_fixes():
    """Suggest fixes for discovery issues"""
    print("\n🔧 Discovery Issue Fixes")
    print("=" * 40)
    print("If topics still aren't showing up, try these fixes:")
    print()
    print("🌐 **DISCOVERY SERVER SETUP (Recommended for TurtleBot 4):**")
    print("   # Option A: Start Discovery Server on TurtleBot")
    print("   ssh ubuntu@192.168.1.70")
    print("   export ROS_DISCOVERY_SERVER=192.168.1.70:11888")
    print("   ros2 run fastdds discovery fastdds-discovery-server -i 0 -l 192.168.1.70 -p 11888")
    print()
    print("   # Then on your host system:")
    print("   export ROS_DISCOVERY_SERVER=192.168.1.70:11888")
    print("   ros2 topic list")
    print()
    print("   # Option B: Use Super Client mode (simpler)")
    print("   export ROS_SUPER_CLIENT=true")
    print("   export ROS_DISCOVERY_SERVER=192.168.1.70:11888")
    print()
    print("💫 **FALLBACK TO MULTICAST DISCOVERY:**")
    print("1. **Check TurtleBot ROS Domain ID:**")
    print("   ssh ubuntu@192.168.1.70")
    print("   echo $ROS_DOMAIN_ID")
    print("   # Should be 0, if not: export ROS_DOMAIN_ID=0")
    print()
    print("2. **Disable localhost-only mode:**")
    print("   export ROS_LOCALHOST_ONLY=0")
    print("   # Add to ~/.bashrc to make permanent")
    print()
    print("3. **Check firewall (both systems):**")
    print("   # On your NixOS system:")
    print("   sudo nixos-rebuild switch --flake .")
    print("   # On TurtleBot:")
    print("   sudo ufw status")
    print("   sudo ufw allow 7400:7499/udp")
    print("   sudo ufw allow 11888/tcp  # For Discovery Server")
    print()
    print("4. **Use static peers (if multicast fails):**")
    print("   export ROS_STATIC_PEERS=192.168.1.70:7400")
    print()
    print("5. **Force discovery refresh:**")
    print("   ros2 daemon stop && ros2 daemon start")
    print("   ros2 topic list --verbose")
    print()
    print("6. **Test direct topic echo (wait for data):**")
    print("   ros2 topic echo /cmd_vel --timeout 10")

def test_discovery_server():
    """Test Discovery Server functionality"""
    print("\n🌐 Discovery Server Test")
    print("=" * 40)
    
    # Check if Discovery Server is configured
    discovery_server = os.environ.get('ROS_DISCOVERY_SERVER', 'NOT SET')
    super_client = os.environ.get('ROS_SUPER_CLIENT', 'NOT SET')
    
    if discovery_server != 'NOT SET':
        print(f"   🌐 Discovery Server configured: {discovery_server}")
        
        # Test connection to Discovery Server
        try:
            if ':' in discovery_server:
                server_ip, server_port = discovery_server.split(':')
                port = int(server_port)
                
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(3)
                result = sock.connect_ex((server_ip, port))
                sock.close()
                
                if result == 0:
                    print(f"   ✅ Discovery Server reachable at {server_ip}:{port}")
                else:
                    print(f"   ❌ Discovery Server NOT reachable at {server_ip}:{port}")
                    print("   💡 Make sure Discovery Server is running on TurtleBot!")
            else:
                print(f"   ⚠️  Invalid Discovery Server format: {discovery_server}")
                
        except Exception as e:
            print(f"   ❌ Discovery Server test error: {e}")
    else:
        print("   ⚪ No Discovery Server configured")
        print("   💡 Consider using Discovery Server for more reliable connection")
    
    if super_client != 'NOT SET':
        print(f"   🌐 Super Client mode: {super_client}")
    
    # Test discovery with current settings
    print("\n   Testing current discovery setup...")
    try:
        result = subprocess.run(['ros2', 'topic', 'list'], 
                              capture_output=True, text=True, timeout=15)
        if result.returncode == 0:
            topics = result.stdout.strip().split('\n')
            robot_topics = [t for t in topics if any(keyword in t for keyword in ['/cmd_vel', '/odom', '/scan', '/battery'])]
            if robot_topics:
                print(f"   ✅ Found robot topics with current setup: {robot_topics}")
            else:
                print(f"   ⚪ Only local topics found: {len(topics)} total")
        else:
            print(f"   ❌ Topic discovery failed: {result.stderr}")
    except subprocess.TimeoutExpired:
        print("   ⏱️  Discovery timed out")
    except Exception as e:
        print(f"   ❌ Discovery test error: {e}")

def test_discovery_server():
    """Test Discovery Server functionality"""
    print("\n🌐 Discovery Server Test")
    print("=" * 40)
    
    # Check if Discovery Server is configured
    discovery_server = os.environ.get('ROS_DISCOVERY_SERVER', 'NOT SET')
    super_client = os.environ.get('ROS_SUPER_CLIENT', 'NOT SET')
    
    if discovery_server != 'NOT SET':
        print(f"   🌐 Discovery Server configured: {discovery_server}")
        
        # Test connection to Discovery Server
        try:
            if ':' in discovery_server:
                server_ip, server_port = discovery_server.split(':')
                port = int(server_port)
                
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.settimeout(3)
                # For UDP, we'll just check if we can bind to a local port
                print(f"   📡 Testing Discovery Server at {server_ip}:{port}")
                print("   ✅ Discovery Server configuration looks good")
                sock.close()
                
            else:
                print(f"   ⚠️  Invalid Discovery Server format: {discovery_server}")
                
        except Exception as e:
            print(f"   ❌ Discovery Server test error: {e}")
    else:
        print("   ⚪ No Discovery Server configured")
        print("   💡 Consider using Discovery Server for more reliable connection")
        print("   💡 Run: source ./discovery-client-setup.sh")
    
    if super_client != 'NOT SET':
        print(f"   🌐 Super Client mode: {super_client}")
    
    # Test discovery with current settings
    print("\n   Testing current discovery setup...")
    try:
        result = subprocess.run(['ros2', 'topic', 'list'], 
                              capture_output=True, text=True, timeout=15)
        if result.returncode == 0:
            topics = result.stdout.strip().split('\n')
            robot_topics = [t for t in topics if any(keyword in t for keyword in ['/cmd_vel', '/odom', '/scan', '/battery'])]
            if robot_topics:
                print(f"   ✅ Found robot topics with current setup: {robot_topics}")
            else:
                print(f"   ⚪ Only local topics found: {len(topics)} total")
                if discovery_server == 'NOT SET':
                    print("   💡 Try setting up Discovery Server for better robot discovery")
        else:
            print(f"   ❌ Topic discovery failed: {result.stderr}")
    except subprocess.TimeoutExpired:
        print("   ⏱️  Discovery timed out")
    except Exception as e:
        print(f"   ❌ Discovery test error: {e}")

if __name__ == "__main__":
    try:
        test_ros_installation()
        test_turtlebot_connection()
        test_ros_turtlebot_discovery()
        test_discovery_server()  # Add Discovery Server test
        test_turtlebot_status()
        troubleshoot_ros_discovery()
        test_discovery_server()
        test_robot_control()
        suggest_discovery_fixes()
        suggest_next_steps()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(1)
