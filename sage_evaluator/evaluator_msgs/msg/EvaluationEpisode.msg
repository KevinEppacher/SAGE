# EvaluationEpisode.msg
string experiment_id
string episode_id
string[] object_names
bool[] success_flags
float32[] detection_confidences
float32[] elapsed_times
float32[] shortest_paths
float32[] actual_paths
geometry_msgs/Point[] goal_poses
geometry_msgs/Point start_pose

# computed values
float32 success_rate                # SR
float32 path_efficiency             # SPL
float32 multi_object_success_rate   # MSR