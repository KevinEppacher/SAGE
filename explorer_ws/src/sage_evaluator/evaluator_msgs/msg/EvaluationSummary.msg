# EvaluationSummary.msg
string experiment_id
string date_time
uint32 total_episodes
float32 success_rate                # SR
float32 avg_spl                     # SPL
float32 avg_multi_object_rate       # MSR
float32 avg_time_per_success
float32 avg_confidence
string ablation_condition           # "OpenFusion=on/off"
string compared_against             # e.g. "VLFM, VLMaps, OneMap, GeFF"
string notes
