# DurableCase UI

A professional Qt-based desktop application for multi-agent robot control and visualization.

## Features

### Left Control Panel
- **Robot Info**: Display current position, route, and status
- **Motion Control**: Intuitive arrow buttons (Forward, Backward, Left, Right, Stop)
- **Mission Control**: Path planning and mission pause/resume
- **Task Management**: Allocate and track tasks across agents

### Right Visualization Panel
- **3D Map Visualization**: OpenGL-based rendering of the operational environment
- **Robot Pose Display**: Real-time robot position and orientation
- **Path Visualization**: Shows planned and executed paths
- **Obstacle Display**: Real-time obstacle map with collision zones
- **System Health Indicator**: Overall system health monitoring

### Integration
- Full ROS2 integration with supervisory control layer
- Real-time system state updates
- Agent status monitoring
- Task allocation via ROS2 services
- Command velocity publishing for robot control

## Building

```bash
cd ~/AANusecase-cooperative-autonomy
colcon build --packages-select durablecase_ui
source install/setup.bash
```

## Running

```bash
# Terminal 1: Start supervisor and agents
ros2 launch supervisor_control supervisor_control.launch.py robot_id:=robot_1

# Terminal 2: Start the UI
ros2 run durablecase_ui durablecase_ui_node
```

## Architecture

### Main Components

1. **MainWindow** - Qt main window integrating all UI panels
2. **MapVisualization** - OpenGL widget for map rendering
3. **ROS2 Integration** - Subscribers and publishers for system communication

### Topics

- **Subscribed**: `/supervisor/system_state`
- **Published**: `/cmd_vel` (command velocity)
- **Services**: `/supervisor/allocate_task`

## Customization

### Styling
Modify button colors and layouts in `createLeftPanel()` and `createRightPanel()` methods.

### Map Display
- Adjust grid size and colors in `drawGrid()`
- Modify obstacle rendering in `drawObstacles()`
- Customize robot visualization in `drawRobot()`

### Update Rate
Change update frequency by modifying the timer interval in `MainWindow` constructor:
```cpp
update_timer_->start(500);  // Currently 500ms
```

## Dependencies

- ROS2 Humble
- Qt5 (Core, Gui, Widgets, OpenGL)
- OpenGL
- GLM (math library)
- supervisor_msgs
- geometry_msgs

## Future Enhancements

- [ ] Agent-specific control panels
- [ ] Real-time trajectory plotting
- [ ] System diagnostics dashboard
- [ ] Mission planning interface
- [ ] Video stream display from onboard cameras
- [ ] Advanced map editing tools
- [ ] Multi-robot coordination visualization

---

**Version**: 0.0.0  
**License**: Apache License 2.0
