# Cooperative Agent Agent

This package contains a Cooperative Agent  (CA) 

## Build

Building this package requires following command (assuming your workspace is in ~/ros2_ws):

```
cd ~/ros2_ws && colcon build --symlink-install --packages-up-to ca
```
Or if you want to rebuild 
```
colcon build --symlink-install --packages-select ca
```
## Usage

The ca can be launched as follows:

```
source ~/ros2_ws/install/setup.bash
ros2 launch lely_ca bringup_launch.py
```


