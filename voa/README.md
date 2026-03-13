# Vehicle Operating Agent

This package contains a Vehicle Operating Agent (VOA) for the Project DurabkeCase

## Build

Building this package requires following command (assuming your workspace is in ~/ros2_ws):

```
cd ~/ros2_ws 
colcon build --symlink-install --packages-select voa
```

## Usage

The VOA can be launched as follows:

```
source ~/ros2_ws/install/setup.bash
ros2 launch voa bringup_launch_robot1.py
ros2 launch voa bringup_launch_robot2.py
```


