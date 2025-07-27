{ config, lib, pkgs, ... }:

{
  # point to the hardware file in ../hardware/
  imports = [ ../hardware/beelink-hardware.nix ];

  nix.settings.experimental-features = [ "nix-command" "flakes" ];

  # System state version (important for system compatibility)
  system.stateVersion = "25.05";

  # Boot loader configuration (UEFI system)
  boot.loader = {
    systemd-boot.enable = true;
    efi.canTouchEfiVariables = true;
  };

  # Basic metadata
  networking.hostName = "beelink";
  time.timeZone     = "America/Chicago";
  i18n.defaultLocale = "en_US.UTF-8";

  # Docker configuration (used for RViz2 containerized solution)
  virtualisation.docker = {
    enable = true;
    enableOnBoot = true;
  };

  # Users (you should update the hashedPassword)
  users.users.louis = {
    isNormalUser = true;
    extraGroups  = [ "wheel" "networkmanager" "docker" ];  # Added docker group
    hashedPassword = "$6$your_hash_here";  # replace with your actual password hash
    
    # SSH configuration for TurtleBot access
    openssh.authorizedKeys.keys = [
      # Add your public keys here if you want to enable SSH access to this machine
    ];
  };

  # SSH service configuration
  services.openssh = {
    enable = true;
    settings = {
      PasswordAuthentication = true;
      PermitRootLogin = "no";
    };
  };

  # Global SSH client configuration
  environment.etc."ssh/ssh_config".text = ''
    # TurtleBot 4 Configuration
    Host turtlebot
        HostName 192.168.1.70
        User ubuntu
        IdentityFile ~/.ssh/id_ed25519_turtlebot
        IdentitiesOnly yes
        ServerAliveInterval 60
        ServerAliveCountMax 3
        StrictHostKeyChecking accept-new
    
    # General SSH settings
    Host *
        ServerAliveInterval 60
        ServerAliveCountMax 3
        ControlMaster auto
        ControlPath ~/.ssh/control-%r@%h:%p
        ControlPersist 10m
  '';

  nixpkgs.config = {
    allowUnfree = true;    # enable any unfree package
    allowUnfreePredicate = pkg:
      builtins.elem (lib.getName pkg) [ "vscode" "code" ];
  };

  # System packages (including ROS base)
  environment.systemPackages = with pkgs; [
    git
    neovim
    wl-clipboard
    vscode
    python3
    python3Packages.pip
    python3Packages.setuptools
    python3Packages.wheel
    python3Packages.pyyaml
    python3Packages.numpy
    python3Packages.lark
    python3Packages.empy
    python3Packages.catkin-pkg
    python3Packages.psutil  # Required by ROS daemon
    
    # Docker tools for containerized RViz2 solution
    docker
    docker-compose
    xorg.xhost  # Required for X11 forwarding to Docker
    
    # Core ROS packages (minimal desktop without visualization)
    rosPackages.jazzy.ros-core
    rosPackages.jazzy.ros-base
    
    # ROS CLI tools
    rosPackages.jazzy.ros2cli
    rosPackages.jazzy.ros2launch
    rosPackages.jazzy.ros2run
    rosPackages.jazzy.ros2node
    rosPackages.jazzy.ros2topic
    rosPackages.jazzy.ros2service
    rosPackages.jazzy.ros2param
    rosPackages.jazzy.ros2pkg
    rosPackages.jazzy.ros2interface
    
    # Python bindings and core libraries
    rosPackages.jazzy.rclpy
    rosPackages.jazzy.rclcpp
    rosPackages.jazzy.rosidl-runtime-py
    rosPackages.jazzy.rpyutils
    rosPackages.jazzy.ament-index-python
    
    # RMW DDS Common
    rosPackages.jazzy.rmw-dds-common
    
    # Message packages
    rosPackages.jazzy.std-msgs
    rosPackages.jazzy.geometry-msgs
    rosPackages.jazzy.sensor-msgs
    rosPackages.jazzy.nav-msgs
    rosPackages.jazzy.rcl-interfaces
    rosPackages.jazzy.builtin-interfaces
    rosPackages.jazzy.action-msgs  # Required by rclpy.action
    rosPackages.jazzy.common-interfaces  # Contains many standard message types
    rosPackages.jazzy.unique-identifier-msgs  # Required by action client
    rosPackages.jazzy.rosgraph-msgs  # Required by time source
    rosPackages.jazzy.type-description-interfaces  # Required by type description service
    rosPackages.jazzy.composition-interfaces  # Required by slam_toolbox and launch
    
    # ROSIDL packages (this should fix rosidl_parser issues)
    rosPackages.jazzy.rosidl-default-runtime
    rosPackages.jazzy.rosidl-default-generators
    rosPackages.jazzy.rosidl-parser
    rosPackages.jazzy.rosidl-adapter
    rosPackages.jazzy.rosidl-generator-c
    rosPackages.jazzy.rosidl-generator-cpp
    rosPackages.jazzy.rosidl-generator-py
    rosPackages.jazzy.rosidl-typesupport-c
    rosPackages.jazzy.rosidl-typesupport-cpp
    rosPackages.jazzy.rosidl-typesupport-fastrtps-c
    rosPackages.jazzy.rosidl-typesupport-fastrtps-cpp
    
    # RMW and DDS implementation
    rosPackages.jazzy.rmw
    rosPackages.jazzy.rmw-implementation
    rosPackages.jazzy.rmw-fastrtps-cpp
    rosPackages.jazzy.rmw-fastrtps-dynamic-cpp
    rosPackages.jazzy.rmw-implementation-cmake
    rosPackages.jazzy.fastrtps
    rosPackages.jazzy.fastcdr
    
    # RMW DDS Common
    rosPackages.jazzy.rmw-dds-common
    
    # Launch packages (required for ros2 daemon)
    rosPackages.jazzy.launch
    rosPackages.jazzy.launch-ros
    rosPackages.jazzy.launch-xml
    rosPackages.jazzy.launch-yaml
    rosPackages.jazzy.launch-testing
    rosPackages.jazzy.osrf-pycommon  # Required by launch package
    
    # Additional useful packages
    rosPackages.jazzy.teleop-twist-keyboard
    rosPackages.jazzy.turtlebot4-msgs
    rosPackages.jazzy.nav2-msgs
    
    # RQT tools (lightweight, safe visualization)
    rosPackages.jazzy.rqt-common-plugins
    rosPackages.jazzy.rqt-graph
    rosPackages.jazzy.rqt-plot
    
    # SLAM and Navigation packages (essential only)
    rosPackages.jazzy.slam-toolbox
    rosPackages.jazzy.nav2-map-server
    rosPackages.jazzy.nav2-lifecycle-manager
    
    # Network tools for robot communication
    nmap
    iputils
    iproute2
    
    # TurtleBot management script
    (pkgs.writeShellScriptBin "turtlebot-ssh-setup" ''
      #!/bin/bash
      # TurtleBot SSH Key Management Script
      
      set -e
      
      TURTLEBOT_IP="192.168.1.70"
      TURTLEBOT_USER="ubuntu"
      SSH_KEY_PATH="$HOME/.ssh/id_ed25519_turtlebot"
      
      case "''${1:-help}" in
        setup)
          echo "🔑 Setting up SSH keys for TurtleBot access..."
          
          # Create .ssh directory if it doesn't exist
          mkdir -p "$HOME/.ssh"
          chmod 700 "$HOME/.ssh"
          
          # Generate SSH key if it doesn't exist
          if [ ! -f "$SSH_KEY_PATH" ]; then
            echo "🔐 Generating new SSH key..."
            ${pkgs.openssh}/bin/ssh-keygen -t ed25519 -f "$SSH_KEY_PATH" \
              -C "turtlebot4-access-$(hostname)" -N ""
            chmod 600 "$SSH_KEY_PATH"
            chmod 644 "$SSH_KEY_PATH.pub"
            echo "✅ SSH key generated: $SSH_KEY_PATH"
          else
            echo "✅ SSH key already exists: $SSH_KEY_PATH"
          fi
          
          # Test connectivity
          echo "🌐 Testing connectivity to TurtleBot..."
          if ! ${pkgs.iputils}/bin/ping -c 1 -W 5 "$TURTLEBOT_IP" >/dev/null 2>&1; then
            echo "❌ Cannot reach TurtleBot at $TURTLEBOT_IP"
            echo "Please ensure TurtleBot is powered on and connected to the network"
            exit 1
          fi
          echo "✅ TurtleBot is reachable"
          
          # Copy public key
          echo "📤 Copying public key to TurtleBot..."
          echo "You will be prompted for the TurtleBot password (default: turtlebot4)"
          
          if ${pkgs.openssh}/bin/ssh-copy-id -i "$SSH_KEY_PATH.pub" "$TURTLEBOT_USER@$TURTLEBOT_IP"; then
            echo "✅ Public key copied successfully"
          else
            echo "❌ Failed to copy public key"
            exit 1
          fi
          
          # Test connection
          echo "🧪 Testing SSH key authentication..."
          if ${pkgs.openssh}/bin/ssh -o ConnectTimeout=5 -o BatchMode=yes \
             -i "$SSH_KEY_PATH" "$TURTLEBOT_USER@$TURTLEBOT_IP" exit 2>/dev/null; then
            echo "✅ SSH key authentication successful!"
            echo ""
            echo "🎉 Setup complete! You can now connect using:"
            echo "   ssh turtlebot"
          else
            echo "❌ SSH key authentication failed"
            exit 1
          fi
          ;;
          
        test)
          echo "🧪 Testing TurtleBot connection..."
          
          # Test ping
          if ${pkgs.iputils}/bin/ping -c 1 -W 5 "$TURTLEBOT_IP" >/dev/null 2>&1; then
            echo "✅ TurtleBot is reachable at $TURTLEBOT_IP"
          else
            echo "❌ TurtleBot is not reachable at $TURTLEBOT_IP"
            exit 1
          fi
          
          # Test SSH
          if [ -f "$SSH_KEY_PATH" ]; then
            if ${pkgs.openssh}/bin/ssh -o ConnectTimeout=5 -o BatchMode=yes \
               -i "$SSH_KEY_PATH" "$TURTLEBOT_USER@$TURTLEBOT_IP" exit 2>/dev/null; then
              echo "✅ SSH key authentication working"
              echo "✅ You can connect with: ssh turtlebot"
            else
              echo "❌ SSH key authentication not working"
              echo "Run 'turtlebot-ssh-setup setup' to configure SSH keys"
            fi
          else
            echo "❌ SSH key not found at $SSH_KEY_PATH"
            echo "Run 'turtlebot-ssh-setup setup' to generate SSH keys"
          fi
          ;;
          
        connect)
          echo "🔌 Connecting to TurtleBot..."
          ${pkgs.openssh}/bin/ssh turtlebot
          ;;
          
        status)
          echo "📊 TurtleBot SSH Status:"
          echo "  TurtleBot IP: $TURTLEBOT_IP"
          echo "  SSH Key Path: $SSH_KEY_PATH"
          echo "  SSH Key Exists: $([ -f "$SSH_KEY_PATH" ] && echo "✅ Yes" || echo "❌ No")"
          echo "  SSH Config: $(grep -q "Host turtlebot" "$HOME/.ssh/config" 2>/dev/null && echo "✅ Configured" || echo "❌ Not configured")"
          ;;
          
        help|*)
          echo "🤖 TurtleBot SSH Management Tool"
          echo ""
          echo "Usage: turtlebot-ssh-setup <command>"
          echo ""
          echo "Commands:"
          echo "  setup    - Generate SSH keys and copy to TurtleBot"
          echo "  test     - Test TurtleBot connectivity and SSH"
          echo "  connect  - Connect to TurtleBot via SSH"
          echo "  status   - Show SSH setup status"
          echo "  help     - Show this help message"
          echo ""
          echo "Examples:"
          echo "  turtlebot-ssh-setup setup"
          echo "  turtlebot-ssh-setup test"
          echo "  turtlebot-ssh-setup connect"
          ;;
      esac
    '')
    
    # Quick TurtleBot connection script
    (pkgs.writeShellScriptBin "turtlebot-connect" ''
      exec ${pkgs.openssh}/bin/ssh turtlebot "$@"
    '')
    
    # TurtleBot keyboard teleop script
    (pkgs.writeShellScriptBin "turtlebot-teleop" ''
      #!/bin/bash
      # TurtleBot 4 Keyboard Teleop Script
      
      echo "🎮 TurtleBot 4 Keyboard Control"
      echo "==============================="
      echo ""
      echo "Starting teleop_twist_keyboard for TurtleBot 4..."
      echo "The TurtleBot 4 uses TwistStamped messages, but teleop_twist_keyboard"
      echo "publishes standard Twist messages, so we remap to /cmd_vel_unstamped."
      echo ""
      echo "Controls:"
      echo "  i/j/k/l - Move forward/left/right/backward"
      echo "  u/o/m/, - Diagonal movements"
      echo "  q/z - Increase/decrease max speeds"
      echo "  w/x - Increase/decrease linear speed only"
      echo "  e/c - Increase/decrease angular speed only"
      echo "  anything else - STOP"
      echo ""
      echo "Press Ctrl+C to quit"
      echo ""
      
      # Check if TurtleBot topics are available
      if ! timeout 5 ros2 topic list | grep -q "/cmd_vel_unstamped"; then
        echo "⚠️  TurtleBot topics not detected. Make sure:"
        echo "   1. TurtleBot is powered on and connected"
        echo "   2. TurtleBot ROS system is running"
        echo "   3. ROS discovery is working: ros2 topic list"
        echo ""
        read -p "Continue anyway? (y/N): " continue_anyway
        if [[ $continue_anyway != [yY] ]]; then
          exit 1
        fi
      fi
      
      echo "🚀 Starting keyboard teleop..."
      exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel_unstamped
    '')
  ];

  # Neovim config
  programs.neovim = {
    enable   = true;
    vimAlias = true;
  };
  environment.variables = {
    EDITOR      = "nvim";
    VISUAL      = "nvim";
    SUDO_EDITOR = "nvim";
    ROS_DISTRO  = "jazzy";
  };

  # ROS 2 environment setup
  environment.sessionVariables = {
    ROS_DOMAIN_ID = "0";
    RMW_IMPLEMENTATION = "rmw_fastrtps_cpp";
    FASTRTPS_DEFAULT_PROFILES_FILE = "";
    ROS_AUTOMATIC_DISCOVERY_RANGE = "SUBNET";  # Match TurtleBot's discovery range
    ROS_DISCOVERY_SERVER = "192.168.1.70:11811";  # TurtleBot Discovery Server
    ROS_SUPER_CLIENT = "true";  # Client mode for Discovery Server
  };

  # Add ROS setup script to shell initialization
  programs.bash.interactiveShellInit = ''
    # Set up ROS environment
    export PYTHONPATH="/run/current-system/sw/lib/python3.12/site-packages:$PYTHONPATH"
    export LD_LIBRARY_PATH="/run/current-system/sw/lib:$LD_LIBRARY_PATH"
    export AMENT_PREFIX_PATH="/run/current-system/sw"
    export CMAKE_PREFIX_PATH="/run/current-system/sw:$CMAKE_PREFIX_PATH"
    export PKG_CONFIG_PATH="/run/current-system/sw/lib/pkgconfig:$PKG_CONFIG_PATH"
    
    # ROS specific environment
    export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
    export ROS_DISTRO="jazzy"
    export ROS_VERSION="2"
    export ROS_DOMAIN_ID="0"
    export ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET"
    
    # Use TurtleBot's Discovery Server for reliable communication
    export ROS_DISCOVERY_SERVER="192.168.1.70:11811"
    export ROS_SUPER_CLIENT="true"
    
    if [ -f /run/current-system/sw/share/ros2-jazzy/setup.bash ]; then
      source /run/current-system/sw/share/ros2-jazzy/setup.bash
    fi
  '';

  # Networking configuration for ROS communication
  networking.firewall = {
    allowedTCPPorts = [ 22 8080 ];  # SSH, TurtleBot web interface
    allowedUDPPorts = [ 7400 7401 7402 7403 11811 ];  # FastRTPS + Discovery Server
    allowedUDPPortRanges = [{ from = 7400; to = 7499; }];
  };

  services.xserver.enable             = true;
  services.xserver.displayManager.gdm.enable = true;
  services.xserver.desktopManager.gnome.enable = true;
}
