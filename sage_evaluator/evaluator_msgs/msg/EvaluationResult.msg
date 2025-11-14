string experiment_id
string episode_id
string prompt

# Metrics
float32 shortest_path_length   # li
float32 actual_path_length     # pi
float32 elapsed_time           # seconds
bool success                   # for SR
string status_reason           # "SUCCESS", "FAILURE", "TIMEOUT", etc.

geometry_msgs/Point start_pose
geometry_msgs/Point goal_pose

builtin_interfaces/Time stamp
