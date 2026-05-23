# AAN Usecase - Cooperative Autonomy

A comprehensive multi-agent robotic system framework built on ROS2, designed for cooperative autonomous control, task allocation, and real-time mission planning across distributed agents.

![Feedback Schematic](https://github.com/tomo1000cmd/AANusecase-cooperative-autonomy/blob/main/feedbackschematic.png?raw=true)
![System Screenshot](https://github.com/tomo1000cmd/AANusecase-cooperative-autonomy/blob/main/Screenshot%20from%202025-02-26%2014-43-38.png?raw=true)

## Overview

This project implements an advanced cooperative autonomy framework featuring multiple specialized agents, behavior trees for intelligent decision-making, supervisory control coordination, and a professional Qt-based UI for real-time monitoring and control. The system is built on ROS2 and is designed for agricultural and industrial multi-robot applications.

## Project Structure

### Core Packages

#### **Agent & Behavior Tree Layer**

- **`bt_agent`** - General-purpose Behavior Tree-based agent framework
  - Provides ROS2 Action interface for behavior tree execution
  - Implements autonomous decision-making logic
  - Build: `colcon build --symlink-install --packages-select bt_agent`

- **`bt_msgs`** - Behavior Tree message definitions
  - Custom ROS2 message and action types for BT execution
  - Action definitions for tree-based task execution

- **`agent_msgs`** - Generic agent communication messages
  - Standard message types for inter-agent communication

#### **Specialized Agents**

- **`ca` (Cooperative Agent)** - Multi-agent coordination
  - Implements cooperative behaviors and task delegation
  - Build: `colcon build --symlink-install --packages-up-to ca`
  - Launch: `ros2 launch lely_ca bringup_launch.py`

- **`fa` (Forecasting Agent)** - Predictive analytics and forecasting
  - Provides forecasting capabilities for mission planning
  - Build: `colcon build --symlink-install --packages-select fa`
  - Launch: `ros2 launch fa bringup_launch.py`

- **`mla` (Machine Learning Agent)** - ML-based decision support
  - Integrates machine learning for enhanced autonomy
  - Dependencies: bt_agent, bt_msgs, agent_msgs, lely_msgs

- **`voa` (Vision/Optimization Agent)** - Visual perception and optimization
  - Handles computer vision and route optimization
  - Dependencies: bt_agent, bt_msgs, agent_msgs, lely_msgs, h2trac_msgs

#### **Supervisory Control Layer**

- **`supervisor_control`** - Distributed supervisory control coordination
  - Centralized management of multi-agent teams
  - Task allocation and scheduling
  - System health monitoring
  - Python and C++ implementations available (supervisory_controldb0.py, supervisory_controldb1.py)

- **`supervisor_msgs`** - Supervisor command and status messages

#### **User Interface**

- **`durablecase_ui`** - Professional Qt-based desktop application
  - **Left Control Panel**: Robot info, motion control, mission control, task management
  - **Right Visualization Panel**: 3D OpenGL map visualization, robot pose display, path visualization, obstacle display, system health monitoring
  - Full ROS2 integration with real-time updates
  - Build: `colcon build --packages-select durablecase_ui`
  - Run: `ros2 run durablecase_ui durablecase_ui_node`

#### **Hardware Interface & Message Packages**

- **`h2trac_msgs`** - H2TRAC hardware interface messages
- **`lely_msgs`** - Lely autonomous system messages
- **`mas_msgs`** - Multi-agent system messages

### Supporting Files

- **`supervisory_controldb0.py`** - Supervisory control implementation (variant 0)
  - Handles robot navigation, docking, field coverage, row-based operations
  - Tkinter-based monitoring interface

- **`supervisory_controldb1.py`** - Supervisory control implementation (variant 1)
  - Alternative supervisory control strategy

### Build Artifacts

- **`build/`** - CMake build outputs
- **`install/`** - Installed packages and setup scripts
- **`log/`** - Build logs and timestamps

## Building the Project

### Prerequisites

- ROS2 (Foxy or later recommended)
- Colcon build system
- CMake 3.5+
- Python 3.8+
- Qt5/Qt6 (for UI package)
- OpenGL development libraries

### Full Build

```bash
cd ~/AANusecase-cooperative-autonomy
colcon build --symlink-install
```

### Build Specific Packages

```bash
# Build behavior tree agent
colcon build --symlink-install --packages-select bt_agent

# Build cooperative agent
colcon build --symlink-install --packages-select ca

# Build UI
colcon build --symlink-install --packages-select durablecase_ui

# Build all dependencies for a package
colcon build --symlink-install --packages-up-to ca
```

## Running the System

### Setup Environment

```bash
cd ~/AANusecase-cooperative-autonomy
source install/setup.bash
```

### Start Supervisor and Agents

```bash
# Terminal 1: Start supervisor control
python3 supervisory_controldb0.py

# Or launch via ROS2
ros2 launch supervisor_control supervisor_control.launch.py robot_id:=robot_1
```

### Launch Agents

```bash
# Terminal 2: Start cooperative agent
ros2 launch lely_ca bringup_launch.py

# Terminal 3: Start forecasting agent
ros2 launch fa bringup_launch.py
```

### Start UI

```bash
# Terminal 4: Start the dashboard
ros2 run durablecase_ui durablecase_ui_node
```

## System Architecture

The system follows a hierarchical control architecture:

```
┌─────────────────────────────────────┐
│   User Interface (durablecase_ui)   │
│  - Real-time visualization          │
│  - Mission control & task allocation│
└────────────────┬────────────────────┘
                 │
┌────────────────▼────────────────────┐
│  Supervisory Control Layer          │
│  - Team coordination                │
│  - Task scheduling                  │
│  - System health monitoring         │
└────────────────┬────────────────────┘
                 │
    ┌────────────┼────────────┐
    │            │            │
┌───▼──┐    ┌───▼──┐    ┌───▼──┐
│  CA  │    │  FA  │    │ MLA  │
│ Agent│    │Agent │    │Agent │
└───┬──┘    └───┬──┘    └───┬──┘
    │           │            │
    └───┬───────┴────────────┘
        │
┌───────▼──────────────────────┐
│  Behavior Tree Execution     │
│  - BT Agent                  │
│  - Decision Logic            │
└───────┬──────────────────────┘
        │
┌───────▼──────────────────────┐
│  Hardware Interface          │
│  - H2TRAC                    │
│  - Lely Systems              │
└──────────────────────────────┘
```

## Key Features

- **Behavior Tree-Based Autonomy**: Flexible, reusable, and intuitive decision-making
- **Multi-Agent Coordination**: Cooperative control and task allocation across robot teams
- **Real-Time Visualization**: Professional Qt-based UI with OpenGL 3D rendering
- **Distributed Supervisory Control**: Scalable coordination for large robot teams
- **Hardware Abstraction**: Support for multiple robot platforms (H2TRAC, Lely)
- **ROS2 Integration**: Modern middleware for robust inter-process communication

## Dependencies & Integration

The system integrates with:
- **ROS2** - Core middleware
- **Nav2** - Navigation and autonomous movement
- **Behavior Tree C++ Library** - BT execution engine
- **Qt5/Qt6** - UI framework
- **OpenGL** - 3D visualization

## License

Apache License 2.0

## Contributors

- Tomoe (Primary maintainer)
- Eric Dortmans
- X. Yanzheng

## Getting Started

1. Clone the repository
2. Install ROS2 and required dependencies
3. Build the workspace: `colcon build --symlink-install`
4. Source the setup: `source install/setup.bash`
5. Launch the UI: `ros2 run durablecase_ui durablecase_ui_node`
6. Launch supervisor and agents in separate terminals

For detailed build and usage instructions, see individual package READMEs in their respective directories.
