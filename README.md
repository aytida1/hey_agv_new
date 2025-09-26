# 🤖 HEY AGV - Autonomous Guided Vehicle System

A comprehensive ROS 2 package for autonomous guided vehicle (AGV) operations with advanced docking capabilities, mapping, and navigation features.

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Requirements](#system-requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Package Structure](#package-structure)
- [Configuration](#configuration)
- [Action Interfaces](#action-interfaces)
- [Nodes Description](#nodes-description)
- [Simulation](#simulation)
- [Contributing](#contributing)
- [License](#license)
- [Authors](#authors)

## 🎯 Overview

HEY AGV is a sophisticated ROS 2 package designed for autonomous guided vehicle operations in warehouse and industrial environments. The system provides comprehensive navigation, docking, and mapping capabilities with AprilTag-based localization for precise positioning and automated docking procedures.

## ✨ Features

- **Autonomous Navigation**: Advanced path planning and obstacle avoidance
- **Precision Docking**: AprilTag-based docking system with multiple dock types
- **Real-time Mapping**: Dynamic environment mapping and localization
- **Multi-sensor Integration**: LiDAR scan merging and sensor fusion
- **Gazebo Simulation**: Complete simulation environment for testing
- **Modular Architecture**: Extensible design with custom action interfaces
- **AMCL Localization**: Adaptive Monte Carlo Localization for robust positioning

## 🔧 System Requirements

- **ROS 2**: Jazzy
- **Operating System**: Ubuntu 24.0 LTS
- **Gazebo**: Harmonic
- **Python**: 3.8+
- **CMake**: 3.8+

### Dependencies

- `rclcpp`
- `nav_msgs`
- `geometry_msgs`
- `tf2_ros`
- `sensor_msgs`
- `apriltag_msgs`
- `nav2_msgs`
- `action_msgs`
- `builtin_interfaces`

## 🚀 Installation

1. **Clone the repository**:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/aytida1/hey_agv_new.git
   ```

2. **Install dependencies**:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package**:
   ```bash
   colcon build --packages-select hey_agv_new
   source install/setup.bash
   ```

## 🎮 Usage

### Launch Simulation

To start the complete AGV simulation environment:

```bash
ros2 launch hey_agv_new launch_sim.launch.py
```

This will launch:
- Gazebo simulation environment
- Robot model and sensors
- Navigation stack
- ROS-Gazebo bridge
- AMCL localization

### Docking Operations

To execute automated docking:

```bash
# Run the docking client
ros2 run hey_agv_new dock.py

# Or send custom docking commands
ros2 action send_goal /dock_robot hey_agv_new/action/DockRobot "{dock_id: 'dock_1', dock_type: 'standard'}"
```

### Manual Control

Control the AGV manually using teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 📁 Package Structure

```
hey_agv_new/
├── action/                     # Custom action definitions
│   ├── DockRobot.action       # Robot docking action
│   ├── LiftAndLock.action     # Lifting mechanism action
│   └── MoveRobot.action       # Movement action
├── config/                     # Configuration files
│   ├── amcl_config.yaml       # AMCL parameters
│   ├── docking_config.yaml    # Docking configuration
│   └── ros_gz_bridge.yaml     # ROS-Gazebo bridge config
├── launch/                     # Launch files
│   └── launch_sim.launch.py   # Main simulation launcher
├── models/                     # 3D models and world files
│   └── layout/                # Environment layout
├── src/                        # Source code
│   ├── dock.py                # Docking action client
│   ├── mapper.cpp             # Frame mapping node
│   ├── scan_merger_v2.cpp     # Advanced scan merging
│   ├── scan_merger.cpp        # LiDAR scan merger
│   ├── tag_publisher.py       # AprilTag pose publisher
│   └── tag_transform.cpp      # Tag transformation node
└── urdf/                       # Robot and world descriptions
    ├── dose_car.urdf.xacro    # Main robot URDF
    ├── test_world_new.sdf     # Simulation world
    └── [various model files]
```

## ⚙️ Configuration

### AMCL Configuration

Edit `config/amcl_config.yaml` to adjust localization parameters:

```yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    # ... other parameters
```

### Docking Configuration

Configure docking behavior in `config/docking_config.yaml`:

```yaml
docking:
  approach_speed: 0.1
  final_approach_distance: 0.3
  dock_timeout: 30.0
```

## 🎬 Action Interfaces

### DockRobot Action

**Goal**:
```
bool use_dock_id
string dock_id
geometry_msgs/PoseStamped dock_pose
string dock_type
float32 max_staging_time
bool navigate_to_staging_pose
```

**Result**:
```
uint16 error_code
string error_description
```

### MoveRobot Action

For autonomous navigation to specified poses.

### LiftAndLock Action

Controls the AGV's lifting mechanism for load handling.

## 🔧 Nodes Description

### Core Nodes

- **`dock.py`**: Handles docking operations and coordinates with navigation stack
- **`mapper.cpp`**: Provides frame transformations and coordinate mapping
- **`tag_publisher.py`**: Processes AprilTag detections and publishes dock poses
- **`scan_merger_v2.cpp`**: Merges multiple LiDAR scans for comprehensive environment perception

### Key Topics

- `/cmd_vel`: Robot velocity commands
- `/scan`: LiDAR scan data
- `/detections`: AprilTag detections
- `/detected_dock_pose`: Computed docking poses
- `/odom`: Odometry information

## 🌐 Simulation

The package includes a complete Gazebo simulation environment featuring:

- Detailed warehouse layout
- Multiple docking stations with AprilTags
- Realistic AGV physics model
- Sensor simulations (LiDAR, cameras)
- Dynamic obstacles and interactive elements

### World Features

- **ASRS Integration**: Automated Storage and Retrieval System
- **Multiple Dock Types**: Standard, delivery, and specialized docks
- **AprilTag Markers**: Precise localization markers (tags 0-3)
- **Realistic Materials**: Proper lighting and textures

## 🤝 Contributing

We welcome contributions! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### Code Style

- Follow ROS 2 coding standards
- Use meaningful variable names
- Comment complex algorithms
- Include unit tests for new features

## 📄 License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## 👥 Authors

- **Aditya**: [aytida1](https://github.com/aytida1) 
- **Krishil**: [Warrior-hound](https://github.com/Warrior-hound)

## 🙏 Acknowledgments

- ROS 2 community for the excellent framework
- Nav2 team for navigation capabilities
- AprilTag developers for robust tag detection
- Gazebo team for simulation environment

## 📞 Support

For questions, issues, or contributions:

- **GitHub Issues**: [Create an issue](https://github.com/aytida1/hey_agv_new/issues)
- **Documentation**: Check the [ROS 2 documentation](https://docs.ros.org/en/jazzy/)
- **Community**: Join the [ROS Discourse](https://discourse.ros.org/)

---

**Happy Robotics! 🤖✨**