# Forcasting Agent

This package contains a Forcasting Agent (FA) 
## Build

Building this package requires following command (assuming your workspace is in ~/ros2_ws):

```
cd ~/ros2_ws && colcon build --symlink-install --packages-select fa
```

## Usage

The fa can be launched as follows:

```
source ~/ros2_ws/install/setup.bash
ros2 launch fa bringup_launch.py
```


