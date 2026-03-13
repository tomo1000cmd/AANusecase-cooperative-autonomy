# Monitoring and Logging Agent

This package contains a Monitoring and Logging Agent (MLA) for the Project Durable CASE.

## Build

Building this package requires following command (assuming your workspace is in ~/ros2_ws):

```
colcon build --symlink-install --packages-select mla
```

## Usage

The mla can be launched as follows:

```
source ~/ros2_ws/install/setup.bash
ros2 launch mla bringup_launch.py
```


