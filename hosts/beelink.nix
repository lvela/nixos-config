{ config, lib, pkgs, ... }:

{
  # point to the hardware file in ../hardware/
  imports = [ ../hardware/beelink-hardware.nix ];

  nix.settings.experimental-features = [ "nix-command" "flakes" ];

  # System state version (important for system compatibility)
  system.stateVersion = "25.05";

  # Basic metadata
  networking.hostName = "beelink";
  time.timeZone     = "America/Chicago";
  i18n.defaultLocale = "en_US.UTF-8";

  # Users (you should update the hashedPassword)
  users.users.louis = {
    isNormalUser = true;
    extraGroups  = [ "wheel" "networkmanager" ];
    hashedPassword = "$6$your_hash_here";  # replace with your actual password hash
  };

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
    
    # Core ROS packages
    rosPackages.jazzy.desktop
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
    
    # Additional useful packages
    rosPackages.jazzy.teleop-twist-keyboard
    rosPackages.jazzy.turtlebot4-msgs
    rosPackages.jazzy.nav2-msgs
    
    # Network tools for robot communication
    nmap
    iputils
    iproute2
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
    
    if [ -f /run/current-system/sw/share/ros2-jazzy/setup.bash ]; then
      source /run/current-system/sw/share/ros2-jazzy/setup.bash
    fi
  '';

  programs.zsh.interactiveShellInit = ''
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
    
    if [ -f /run/current-system/sw/share/ros2-jazzy/setup.bash ]; then
      source /run/current-system/sw/share/ros2-jazzy/setup.bash
    fi
  '';

  # Networking & desktop
  networking.networkmanager.enable = true;
  
  # Allow ROS multicast traffic
  networking.firewall = {
    enable = true;
    allowedTCPPorts = [ 22 8080 11311 ];
    allowedUDPPorts = [ 7400 7401 7402 7403 ];
    allowedUDPPortRanges = [
      { from = 7400; to = 7499; }  # FastRTPS discovery
    ];
  };
  
  services.xserver.enable             = true;
  services.xserver.displayManager.gdm.enable = true;
  services.xserver.desktopManager.gnome.enable = true;

  # Bootloader
  boot.loader.systemd-boot.enable       = true;
  boot.loader.efi.canTouchEfiVariables  = true;
}

