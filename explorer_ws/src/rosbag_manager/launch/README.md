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
ros2 launch rosbag_manager play_rosbag.launch.py   bag_path:=src/rosbag_manager/bag/bag_2025-05-20_19-00-55_HaxA7YrQdEC_pointcloud  rate:=3.0   loop:=true
ros2 launch rosbag_manager play_rosbag.launch.py   bag_path:=src/rosbag_manager/bag/bag_2025-05-25_09-16-33_Frontiers_Inflation  rate:=3.0   loop:=true
ros2 launch rosbag_manager play_rosbag.launch.py   bag_path:=/app/src/rosbag_manager/bag/bag_2025-06-08_18-22-29_HaxA7YrQdEC  rate:=2.0   loop:=true

```

## Visualize data
```bash
ros2 launch rosbag_manager rviz.launch.py
```

## Navigate Robot
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## SLAM + Local Costmap for Frontier Exploration
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

ros2 launch nav2_bringup bringup_launch.py slam:=True use_sim_time:=True autostart:=True map:=/dev/null
ros2 launch nav2_bringup bringup_launch.py slam:=True use_sim_time:=True autostart:=True map:=/dev/null params_file:=/app/nav2_params.yaml

sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-msgs ros-humble-turtlebot3-bringup ros-humble-turtlebot3-gazebo
sudo apt install ros-humble-turtlebot3 ros-humble-turtlebot3-msgs ros-humble-turtlebot3-bringup ros-humble-turtlebot3-gazebo
sudo apt install ros-humble-turtlebot3-simulations
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
export TURTLEBOT3_MODEL=waffle

```
```bash
ros2 bag record \
  -o /app/src/rosbag_manager/bag/bag_$(date +%Y-%m-%d_%H-%M-%S)_Frontiers_Inflation \
  --compression-mode file \
  --compression-format zstd \
  /behavior_server/transition_event \
  /clock \
  /cmd_vel \
  /cmd_vel_nav \
  /cmd_vel_teleop \
  /controller_server/transition_event \
  /cost_cloud \
  /global_costmap/costmap \
  /global_costmap/costmap_raw \
  /global_costmap/costmap_updates \
  /global_costmap/footprint \
  /global_costmap/global_costmap/transition_event \
  /global_costmap/published_footprint \
  /goal_pose \
  /imu \
  /initialpose \
  /joint_states \
  /local_costmap/clearing_endpoints \
  /local_costmap/costmap \
  /local_costmap/costmap_raw \
  /local_costmap/costmap_updates \
  /local_costmap/footprint \
  /local_costmap/local_costmap/transition_event \
  /local_costmap/published_footprint \
  /local_costmap/voxel_grid \
  /local_plan \
  /map \
  /map_metadata \
  /map_saver/transition_event \
  /map_updates \
  /marker \
  /odom \
  /parameter_events \
  /performance_metrics \
  /plan \
  /plan_smoothed \
  /planner_server/transition_event \
  /pose \
  /preempt_teleop \
  /received_global_plan \
  /robot_description \
  /rosout \
  /scan \
  /slam_toolbox/feedback \
  /slam_toolbox/graph_visualization \
  /slam_toolbox/scan_visualization \
  /slam_toolbox/update \
  /smoother_server/transition_event \ 
  /speed_limit \
  /tf \
  /tf_static \
  /transformed_global_plan \
  /velocity_smoother/transition_event \
  /waypoint_follower/transition_event
  ```
  
