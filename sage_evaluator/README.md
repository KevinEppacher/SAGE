# Filestructure:
```bash
datasets/
└─ matterport_isaac/
   └─ {scene_name}/
        ├─ assets/
        │   ├─ scene.usd
        │   ├─ textures/
        │   └─ objects.json
        │
        ├─ annotations/
        │   ├─ v1.0/
        │   │   ├─ semantic_map.ply
        │   │   ├─ semantic_classes.json
        │   │   ├─ robot_start_pose.json
        │   │   └─ calibration/camera_info.yaml
        │   ├─ v1.1/
        │   │   ├─ semantic_map.ply
        │   │   ├─ semantic_classes.json
        │   │   ├─ robot_start_pose.json
        │   │   └─ changelog.txt
        │   └─ current -> v1.1/
        │
        ├─ episodes/
        │   ├─ 0001_{timestamp}/
        │   │   ├─ bag/
        │   │   ├─ trajectory.json
        │   │   ├─ prompts.txt
        │   │   ├─ metrics.csv
        │   │   ├─ run_summary.json
        │   │   ├─ detections/
        │   │   │   ├─ chair/
        │   │   │   │   ├─ detection_0001.png      # screenshot or cropped detection
        │   │   │   │   ├─ detection_0002.png
        │   │   │   │   ├─ detection_meta.json     # confidence, timestamp, pose
        │   │   │   │   └─ detection_cam_pose.json # camera pose at detection
        │   │   │   ├─ table/
        │   │   │   │   ├─ detection_0001.png
        │   │   │   │   └─ detection_meta.json
        │   │   │   └─ ...
        │   │   └─ bt_trace/
        │   │       ├─ raw_log.json
        │   │       ├─ aggregated.json
        │   │       └─ sankey_data.json
        │   └─ summary.csv
        │
        ├─ results/
        │   ├─ onemap_baseline/
        │   ├─ vlfm_baseline/
        │   └─ openfusion/
        │
        ├─ scene_metadata.yaml
        │
        └─ logs/
            ├─ rosout.log
            ├─ isaac_sim_stdout.txt
            └─ system_usage.json
```

metadata.yaml
```yaml
scene_id: "matterport_isaac/living_room_02"
scene_version: "v1.1"
dataset_version: "2025.11"
description: "Medium-sized apartment living room with sofa, TV shelf, and dining table."
source: "Matterport3D, converted to Isaac Sim via USD export"
sim_version: "Isaac Sim 2025.1"
annotation_author: "Fixi Hartmann"
annotation_method: "OpenFusion + manual refinement"
last_updated: "2025-11-06"
alignment_reference: "/map"
```

raw_log.json
```json
[
  {"timestamp": 0.2, "node": "NavigateToPose", "status": "RUNNING"},
  {"timestamp": 5.3, "node": "IsDetected", "status": "SUCCESS"},
  {"timestamp": 8.1, "node": "IsGoalReached", "status": "FAILURE", "description": "Stopped too far from goal"},
  {"timestamp": 10.0, "node": "RetryBehavior", "status": "SUCCESS"}
]
```

aggregated.json
```json
{
  "success": false,
  "nodes": {
    "IsDetected": {"count": 1, "success_rate": 0.95},
    "NavigateToPose": {"count": 1, "success_rate": 0.75}
  },
  "failure_reason": "Stopped too far from goal"
}
```

sankey_data.json
```json
{
  "labels": ["Failures", "Called STOP", "Timeout", "Saw goal", "Stopped too far", "Stopped at wrong object"],
  "values": [70.6, 45.1, 25.5, 24.5, 13.1, 20.6],
  "links": [
    {"source": "Failures", "target": "Called STOP", "value": 45.1},
    {"source": "Failures", "target": "Timeout", "value": 25.5},
    {"source": "Called STOP", "target": "Stopped too far", "value": 13.1},
    {"source": "Called STOP", "target": "Stopped at wrong object", "value": 20.6}
  ]
}
```

ros2 topic pub --once /evaluator/prompt multimodal_query_msgs/msg/SemanticPrompt "{text_query: chair}"
ros2 topic pub --once /zero_shot_prompt multimodal_query_msgs/msg/SemanticPrompt "{text_query: a door leading to a chair}"
ros2 topic pub --once /user_prompt multimodal_query_msgs/msg/SemanticPrompt "{text_query: chair}"

a door leading to a chair