# Fired at the beginning of each prompt.
string experiment_id
string episode_id
string scene
string current_prompt
multimodal_query_msgs/SemanticPromptArray prompt_list
geometry_msgs/Point start_pose
geometry_msgs/Point goal_pose                               # optional (can be zeroed before planning)
string save_path                                            # directory for writing this prompt's data
builtin_interfaces/Time stamp
