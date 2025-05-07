# ROS2 Bag Manager Pipeline
## Record data with a argument as world name (eg. HaxA7YrQdEC, default name=default_world)
```bash
ros2 launch rosbag_manager record_bag.launch.py world_name:=HaxA7YrQdEC
```
TODO: Does not work well for large rosbags, because it kills the process when it is not finished compressing
Terminal command:
```bash
ros2 bag record \
  -o /app/src/rosbag_manager/bag/bag_$(date +%Y-%m-%d_%H-%M-%S)_HaxA7YrQdEC \
  --compression-mode file \
  --compression-format zstd \
  /camera_info \
  /clock \
  /depth \
  /depth_pcl \
  /map \
  /map_metadata \
  /map_updates \
  /odom \
  /parameter_events \
  /pose \
  /rgb \
  /rosout \
  /tf \
  /tf_static
  ```

Bevore recording, make sure to launch slam_toolbox
```bash
ros2 launch slam_toolbox online_async_launch.py
```

## Play data with argumenet of bag path, rate and loop
```bash
ros2 launch rosbag_manager play_rosbag.launch.py   bag_path:=/app/src/rosbag_manager/bag/bag_2025-05-07_10-14-34_HaxA7YrQdEC   rate:=1.0   loop:=true
```

## Visualize data
```bash
ros2 launch rosbag_manager rviz.launch.py
```

## Navigate Robot
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```