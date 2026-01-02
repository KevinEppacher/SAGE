# 🧭 Dataset Structure Overview

This document explains the organization and purpose of all files and directories within the evaluation dataset for **Matterport–Isaac** scenes.  
Each scene folder is a *self-contained evaluation environment*, including simulation assets, annotations, recorded episodes, metrics, visual detections, and logs.

---

## Filestructure

```bash
sage_datasets/
└─ matterport_isaac/
   └─ {scene_name}/
        ├─ assets/
        │   ├─ scene.usd
        │   ├─ textures/
        │   └─ classes.json
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
        │   ├─ 001/
        │   │   ├─ bag/
        │   │   ├─ robot_start_pose.json
        │   │   ├─ prompts.txt
        │   │   ├─ metrics.csv
        │   │   ├─ run_summary.json
        │   │   ├─ detections/
        │   │   │   ├─ chair/
        │   │   │   │   ├─ detection_0001.png
        │   │   │   │   ├─ trajectory.json
        │   │   │   │   ├─ spl.json
        │   │   │   │   └─ sr.json
        │   │   │   ├─ table/
        │   │   │   │   ├─ detection_0001.png
        │   │   │   │   ├─ trajectory.json
        │   │   │   │   ├─ spl.json
        │   │   │   │   └─ sr.json
        │   │   │   └─ ...
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

---

## 📁 Folder Descriptions

### **`assets/`**
Contains the simulation assets used by Isaac Sim or another simulator.

| File | Description |
|------|--------------|
| `scene.usd` | The full USD scene describing the environment geometry, lighting, and materials. |
| `textures/` | Folder containing texture maps referenced in the USD scene. |
| `objects.json` | List of interactable or labeled objects within the scene. Example: |
| | ```json
{"objects": [{"id": 1, "name": "chair", "pose": [1.2, 0.4, 0.0]}, {"id": 2, "name": "table"}]}
``` |

---

### **`annotations/`**
Stores semantic and geometric reference data for evaluation.  
Multiple versions (`v1.0`, `v1.1`, …) allow iterative updates or refinements.

| File | Description |
|------|--------------|
| `semantic_map.ply` | Full 3D colored point cloud annotated by class ID. |
| `semantic_classes.json` | Maps numerical class IDs to human-readable labels. |
| `robot_start_pose.json` | Stores the initial robot pose in the scene coordinate frame. |
| `calibration/camera_info.yaml` | Camera intrinsics and distortion parameters for reprojection. |
| `changelog.txt` | Notes describing differences between versions (added classes, fixes). |
| `current -> v1.1/` | Symlink to the currently active annotation version. |

---

### **`episodes/`**
Each **episode** corresponds to one run (typically one prompt).  
Stores all runtime data, detections, and behavior-tree traces.

#### Example hierarchy:
```bash
episodes/0001_20251106_173245/
├─ bag/
├─ trajectory.json
├─ prompts.txt
├─ metrics.csv
├─ run_summary.json
├─ detections/
│   ├─ chair/
│   │   ├─ detection_0001.png
│   │   ├─ detection_meta.json
│   │   └─ detection_cam_pose.json
│   └─ table/
│       ├─ detection_0001.png
│       └─ detection_meta.json
└─ bt_trace/
    ├─ raw_log.json
    ├─ aggregated.json
    └─ sankey_data.json
```

#### Example `detection_meta.json`
```json
{
  "timestamp": 1762418823.123,
  "confidence": 0.84,
  "pose": [3.21, 1.07, 0.52],
  "frame_id": "map",
  "source": "live_inference"
}
```

#### Example `trajectory.json`
```json
[
  {"time": 0.0, "pose": [0.0, 0.0, 0.0, 0.0]},
  {"time": 2.0, "pose": [1.1, 0.3, 0.0, 0.05]},
  {"time": 4.0, "pose": [2.4, 1.1, 0.0, 0.10]}
]
```

#### Example `metrics.csv`
| metric | value |
|--------|-------:|
| SR | 1.0 |
| SPL | 0.91 |
| PR | 0.85 |
| Time | 42.3 |

#### Example `run_summary.json`
```json
{
  "scene": "living_room_02",
  "prompt": "Find chair",
  "success": true,
  "start_pose": [0.0, 0.0, 0.0],
  "end_pose": [3.4, 1.7, 0.0],
  "distance_traveled": 6.12
}
```

#### Behavior Tree Logs
`bt_trace/` contains data for failure/success analysis:

- **`raw_log.json`** — chronological sequence of node states.  
- **`aggregated.json`** — summarized success/failure per node.  
- **`sankey_data.json`** — aggregated high-level behavior tree outcomes, used for Sankey diagram visualizations.

---

### **`results/`**
Aggregated evaluation outputs per model or baseline.

| Subfolder | Description |
|------------|--------------|
| `onemap_baseline/` | Metrics from baseline OneMap system. |
| `vlfm_baseline/` | Metrics from Vision-Language Frontier Maps. |
| `openfusion/` | Results from OpenFusion evaluation, including plots. |

Example:
```bash
results/openfusion/
├─ metrics_overview.csv
└─ plots/
   ├─ spl_curve.png
   ├─ success_rate.png
   └─ qualitative.gif
```

---

### **`scene_metadata.yaml`**
Scene-level metadata file describing the dataset version, simulation source, and annotation context.

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

---

### **`logs/`**
Contains raw system outputs for debugging and reproducibility.

| File | Description |
|------|--------------|
| `rosout.log` | ROS2 console output from the run. |
| `isaac_sim_stdout.txt` | Simulation console log. |
| `system_usage.json` | CPU, GPU, and memory utilization trace during episode. Example: |
| | ```json
{"cpu": 68.2, "gpu": 92.1, "mem_used": 11.4}
``` |

---

## 🧩 Versioning and Reproducibility Notes

- **Annotation versions** (`v1.0`, `v1.1`, …) capture updates to ground-truth or labeling schemes.  
  Each version should include a `changelog.txt` documenting differences.  
- The symbolic link `annotations/current` always points to the active reference version used for evaluation.  
- Each episode folder is immutable once finalized — reruns produce a new timestamped directory.
- Results and metrics are aggregated scene-wise and model-wise for consistent comparison.

---

This documentation ensures anyone opening your dataset can immediately understand:
- What each file is for  
- How results are reproducible  
- Where visual and behavioral evidence is stored  

It’s publication-ready and aligned with robotics reproducibility standards (Habitat-Lab, VLN-CE, AI2-THOR, etc.).


ros2 topic pub --once /evaluator/prompt multimodal_query_msgs/msg/SemanticPrompt "{text_query: chair}"
ros2 topic pub --once /zero_shot_prompt multimodal_query_msgs/msg/SemanticPrompt "{text_query: a door leading to a chair}"
ros2 topic pub --once /user_prompt multimodal_query_msgs/msg/SemanticPrompt "{text_query: chair}"

ros2 topic pub --once /user_prompt multimodal_query_msgs/msg/SemanticPrompt \
"{text_query: chair}" \
--qos-durability transient_local \
--qos-reliability reliable \
--qos-history keep_last \
--qos-depth 1
```

# Available Matterport Scenes (VLFM uses these scenes):

| VLFM file               | Matterport scene directory |
| ----------------------- | -------------------------- |
| `TEEsavR23oF.json(.gz)` | `00800-TEEsavR23oF`        |
| `wcojb4TFT35.json.gz`   | `00802-wcojb4TFT35`        |
| `svBbv1Pavdk.json.gz`   | `00813-svBbv1Pavdk`        |
| `p53SfW6mjZe.json.gz`   | `00814-p53SfW6mjZe`        |
| `mL8ThkuaVTM.json.gz`   | `00820-mL8ThkuaVTM`        |
| `Dd4bFSTQ8gi.json.gz`   | `00824-Dd4bFSTQ8gi`        |
| `QaLdnwvtxbs.json.gz`   | `00829-QaLdnwvtxbs`        |
| `qyAac8rV8Zk.json.gz`   | `00832-qyAac8rV8Zk`        |
| `q3zU7Yy5E5s.json.gz`   | `00835-q3zU7Yy5E5s`        |
| `zt1RVoi7PcG.json.gz`   | `00839-zt1RVoi7PcG`        |
| `DYehNKdT76V.json.gz`   | `00843-DYehNKdT76V`        |
| `ziup5kvtCCR.json.gz`   | `00848-ziup5kvtCCR`        |
| `5cdEh9F2hJL.json.gz`   | `00853-5cdEh9F2hJL`        |
| `bxsVRursffK.json.gz`   | `00873-bxsVRursffK`        |
| `4ok3usBNeis.json.gz`   | `00877-4ok3usBNeis`        |
| `mv2HUxq3B53.json.gz`   | `00876-mv2HUxq3B53`        |
| `XB4GS9ShBRE.json.gz`   | `00878-XB4GS9ShBRE`        |
| `Nfvxx8J5NCo.json.gz`   | `00880-Nfvxx8J5NCo`        |
| `6s7QHgap2fW.json.gz`   | `00890-6s7QHgap2fW`        |
| `cvZr5TUy5C5.json.gz`   | `00891-cvZr5TUy5C5`        |

### 00800-TEEsavR23oF:
### 00802-wcojb4TFT35:
### 00813-svBbv1Pavdk:
### 00814-p53SfW6mjZe:
### 00824-Dd4bFSTQ8gi:
### 00829-QaLdnwvtxbs:
### 00839-zt1RVoi7PcG:
### 00848-ziup5kvtCCR:
### 00853-5cdEh9F2hJL:
### 00876-mv2HUxq3B53:
### 00880-Nfvxx8J5NCo:
### 00891-cvZr5TUy5C5:

## RQ1 - SR/SPL across different scenes compared to VLFM, OneMap, Pigeon,...:
Metrics: Performance is quantified in terms of task success and path efficiency, mea-
sured through Success Rate (SR), Success weighted by Path Length (SPL), and Multi-
Object Success Rate (MSR) relative to representative state-of-the-art systems such as
OneMap, VLFM, and Pigeon.

## RQ2 - Exploration vs Exploitation trade-off analysis:
Metrics: The weighting factor between exploration and memory is varied during graph
node fusion to assess impacts on SR and SPL, identifying optimal trade-offs between
reactivity and exploitation.

### 00800-TEEsavR23oF:
**Prompts:**
```bash
"chair, armchair, couch",
"bed",
"oven and stove",
"sofa",
"refrigerator"
```

## RQ3 - SR/SPL by increasing Semantic Map Quality Granularity:
Metrics: The semantic granularity in the 3D semantic mapper is varied while adjusting
exploration weight to evaluate effects on SR and SPL.

## RQ4 - detection robustness for COCO classes, open-vocabulary classes, and zero-shot classes:
Metrics: Precision, Recall, F1-Score, Confusion Matrix, and SR under different fusion
weight configurations across COCO, open-vocabulary, and zero-shot classes.

## RQ5 - Real-time performance and robustness under sensor noise:
Metrics: Frames Per Second (FPS), GPU/CPU usage, inference latency, and detection
stability under sensor noise during physical deployment on a mobile robot.
