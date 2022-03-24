# Merge Point Cloud Package

This is repository of merge_point_cloud package for ROS 2 foxy environment.

## Installation
```bash
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ git clone https://github.com/hskim617/merge_point_cloud.git
```

## Build
```bash
$ cd ~/ros2_ws
$ colcon build --symlink-install --packages-select merge_point_cloud
$ source ~/ros2_ws/install/local_setup.bash
```

## Run the launch file
You might change the param/config.yaml file.
```bash
$ ros2 launch merge_point_cloud merge_point_cloud.launch.py
```

## License
MIT License
