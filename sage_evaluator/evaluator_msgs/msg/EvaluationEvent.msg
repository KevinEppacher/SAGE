# EvaluationEvent.msg
# Emitted by BT node when something meaningful happens during an experiment.

string phase                                                # "START", "SUCCESS", "FAILURE", "TIMEOUT"
string scene
string experiment_id                                        # unique ID, e.g. "EXP_001"
string episode_id                                           # sub-ID for multi-object runs
multimodal_query_msgs/SemanticPromptArray prompt_list       # e.g. "chair", "bed", "lamp"
string save_path                                            # directory path, where images are saved
float32 confidence                                          # last detection confidence, if available
float32 elapsed_time                                        # seconds since episode start
float32 shortest_path_length                                # li (m) from planner (optional)
float32 actual_path_length                                  # pi (m) traveled by robot
bool success                                                # convenience flag for logging
string reason                                               # e.g. "no_graph_nodes", "timeout", "navigation_fail"
geometry_msgs/Point goal_pose                               # goal coordinate in map frame
geometry_msgs/Point start_pose                              # starting point of this episode
