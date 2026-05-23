# DurableCase UI - Quick Start Guide

## Overview

The DurableCase UI is a professional Qt5-based desktop application that provides comprehensive control and monitoring of your multi-agent robotic system integrated with the supervisory control layer.

## Installation

### Prerequisites
```bash
# Install Qt5 development libraries
sudo apt-get install qt5-qmake qt5-default libqt5opengl5-dev

# Install OpenGL development libraries
sudo apt-get install libgl1-mesa-dev

# Install GLM (OpenGL Mathematics)
sudo apt-get install libglm-dev
```

### Build
```bash
cd ~/AANusecase-cooperative-autonomy
colcon build --packages-select durablecase_ui
source install/setup.bash
```

## Usage

### Start the System

**Terminal 1 - Launch Supervisor and Agents**
```bash
# Source ROS2
source /opt/ros/humble/setup.bash
source ~/AANusecase-cooperative-autonomy/install/setup.bash

# Launch supervisor control layer
ros2 launch supervisor_control supervisor_control.launch.py robot_id:=robot_1
```

**Terminal 2 - Launch Agents (VOA, MLA, CA, FA)**
```bash
source /opt/ros/humble/setup.bash
source ~/AANusecase-cooperative-autonomy/install/setup.bash

# Launch multi-agent system (create a launch file or run individually)
ros2 run voa voa_bt &
# Add other agents as needed
```

**Terminal 3 - Launch UI**
```bash
source /opt/ros/humble/setup.bash
source ~/AANusecase-cooperative-autonomy/install/setup.bash

ros2 run durablecase_ui durablecase_ui_node
```

## UI Controls

### Left Panel - Robot Control

#### Robot Info Section
- **Robot ID**: Displays the robot identifier
- **Current Position**: X, Y coordinates and angle θ in real-time
- **Current Route ID**: Active route identifier
- **Current Status**: Mission state (Idle, Active, Error, etc.)
- **Progress**: Visual progress bar showing mission completion
- **Start Time**: Mission start timestamp
- **Velocity**: Current linear and angular velocities

#### Motion Control Section
- **↑ Forward**: Move robot forward (linear velocity +0.5 m/s)
- **↓ Backward**: Move robot backward (linear velocity -0.5 m/s)
- **← Left**: Rotate counter-clockwise (angular velocity +0.5 rad/s)
- **Right →**: Rotate clockwise (angular velocity -0.5 rad/s)
- **STOP**: Emergency stop (all velocities: 0)

#### Mission Control Section
- **Set path**: Define navigation path (integrated with path planner)
- **Stop path**: Abort current path following
- **Pause**: Pause current mission without stopping motors
- **Resume**: Continue paused mission

#### Task Management Section
- **Task UID**: Input field for task unique identifier
- **Allocate Task**: Request task execution from supervisor
- **Task List**: Table showing current, pending, and completed tasks

### Right Panel - Visualization

#### Map View
- **Grid**: Background reference grid (5m intervals)
- **Robot (Green)**: Current robot position and orientation
  - Rectangle shows robot body
  - Red triangle shows direction
- **Obstacles (Magenta)**: Current obstacles and restricted zones
- **Path (Cyan)**: Planned and executed navigation path
- **Goal (Yellow)**: Target destination marker

#### Camera Controls
- **Zoom**: Mouse wheel to zoom in/out
- **Pan**: Click and drag to pan the view
- **Follow**: Automatically center on robot (toggle option)

#### System Health Bar
- **Green**: All systems healthy (>60%)
- **Orange**: Degraded operation (30-60%)
- **Red**: Emergency state (<30%)

## API Integration

The UI communicates with your system via these ROS2 interfaces:

### Subscriptions
- `/robot_1/supervisor/system_state` (supervisor_msgs/SystemState)
  - Receives system-wide status updates
  - Includes: system mode, active tasks, agent count, health metrics

### Publications
- `/cmd_vel` (geometry_msgs/Twist)
  - Sends motion commands from motion control buttons
  - Contains: linear velocity (x, y, z) and angular velocity (roll, pitch, yaw)

### Services
- `/robot_1/supervisor/allocate_task` (supervisor_msgs/AllocateTask)
  - Requests task allocation to specific agent
  - Provides feedback on allocation success/failure

## Workflow Examples

### Example 1: Simple Forward Motion

1. Click **"↑ Forward"** button
2. Observe robot moving forward in visualization
3. Monitor velocity in "Current Velocity" section
4. Click **"STOP"** to halt motion

### Example 2: Navigate to Goal

1. Enter task UID in "Task Management" section
2. Click **"Set path"** to define navigation route
3. Click **"Allocate Task"** to assign to VOA
4. Monitor progress in the map visualization
5. Watch task status update in "Task List" table

### Example 3: Emergency Response

1. Observe system health is in RED (<30%)
2. System automatically switches to EMERGENCY mode
3. Click **"STOP"** to halt all motion
4. Check failed agents in system status
5. Manual intervention available for critical tasks

## Troubleshooting

### UI Doesn't Show Robot Position
- Verify supervisor control layer is running
- Check that agents are publishing AgentStatus messages
- Monitor ROS2 topics: `ros2 topic list | grep supervisor`

### Motion Commands Have No Effect
- Verify robot is subscribed to `/cmd_vel`
- Check that VOA agent is running and active
- Ensure system is not in LOCAL_ONLY mode (centralized coordination lost)

### Visualization Not Updating
- Restart both supervisor and UI
- Verify OpenGL drivers are properly installed
- Check GPU support: `glxinfo | grep "OpenGL version"`

### Build Errors Related to Qt5
```bash
# If Qt5 not found, install:
sudo apt-get install qtbase5-dev qt5-qmake
```

## Performance Optimization

The UI updates at 500ms intervals to balance responsiveness with CPU load. Adjust in code:

```cpp
update_timer_->start(500);  // Change to desired milliseconds
```

## Advanced Features

### Custom Themes
Modify button color schemes in `createLeftPanel()`:
```cpp
stop_btn_->setStyleSheet("background-color: red; color: white; font-weight: bold;");
```

### Extended Data Display
Add new data fields to Robot Info section:
```cpp
QLabel *battery_label_ = new QLabel("Battery: 95%", this);
info_layout->addRow("Battery Level:", battery_label_);
```

### Real-time Plotting
Integrate plotting library (e.g., QCustomPlot) for trajectory visualization

## Support

For issues or feature requests, refer to:
- Supervisor Control docs: `SUPERVISORY_CONTROL_INTEGRATION.md`
- VOA integration: `voa/README.md`
- ROS2 Humble docs: https://docs.ros.org/en/humble/

---

**Last Updated**: April 9, 2026  
**Version**: 0.0.0
