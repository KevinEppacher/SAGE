# **SAGE (Semantic-Aware Guided Exploration with persistent Memory)**

SAGE is a ROS 2-based autonomous exploration framework that combines frontier exploration, semantic 3D mapping, zero-shot object detection, and vision-language reasoning to search for user-specified objects in simulated indoor environments.

Frameworks:  
![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue?logo=ros)
![Nav2](https://img.shields.io/badge/Nav2-Frontier%20Exploration-brightgreen?logo=ros)
![Isaac Sim](https://img.shields.io/badge/Isaac--Sim-Simulation-lightgrey?logo=nvidia)
![OpenFusion](https://img.shields.io/badge/OpenFusion-3D%20Semantic%20Mapping-purple)  

Technologies:  
![YOLO-E](https://img.shields.io/badge/YOLO--E-Zero--Shot%20Detection-red)
![BLIP2](https://img.shields.io/badge/BLIP2-Vision--Language%20Model-orange)
![SEEM](https://img.shields.io/badge/SEEM-Segment%20Everything%20Everywhere-blueviolet)  

Programming Languages:  
![Python](https://img.shields.io/badge/Python-3.10-yellow?logo=python) 
![C++](https://img.shields.io/badge/C%2B%2B-17-lightgrey?logo=c%2B%2B)

Development Tools:  
![Docker](https://img.shields.io/badge/Docker-Reproducibility-blue?logo=docker)
![Cuda](https://img.shields.io/badge/CUDA-Acceleration-red?logo=nvidia)
![CUDA](https://img.shields.io/badge/CUDA-12.1.1-76b900?logo=nvidia)
![CUDA](https://img.shields.io/badge/CUDA-11.7.1-76b900?logo=nvidia)


## Overview
SAGE (Semantic-Aware Guided Exploration with persistent Memory) is a cutting-edge framework designed to enable autonomous robots to explore and understand their environments effectively. By integrating advanced semantic mapping, zero-shot object detection, and vision-language models, SAGE allows robots to navigate complex environments while building a persistent memory of their surroundings. This framework is built on ROS 2 Humble and leverages state-of-the-art technologies such as OpenFusion for 3D semantic mapping, YOLO-E for zero-shot object detection, BLIP2 for vision-language understanding, SEEM for comprehensive segmentation, and Nav2 for frontier exploration. SAGE is also compatible with NVIDIA Isaac Sim for realistic simulation and testing. With its modular architecture and robust capabilities, SAGE represents a significant advancement in the field of autonomous robotics, enabling more intelligent and efficient exploration in a wide range of applications.

## Key Architecture

![](./docs/sage_architecture_simplified.png)

SAGE consists of the following main modules:

| Module | Description |
|---|---|
| Explorer container | Runs navigation and mapping, frontier exploration, behavior tree execution, and high-level task control. |
| Detection container | Runs YOLO-E-based zero-shot object detection and pointcloud processing. |
| Exploitation container | Runs OpenFusion with SEEM as a semantic mapper and pointcloud processing. |
| LAVIS container | Runs BLIP2-based vision-language cosine similarity scoring as a ros2 service. |
| Isaac Sim container | Provides the simulation environment and robot interface. |

## Installation

1. Install Docker. See [installation instructions](./docs/install.md).

2. Clone the repository and start the containers.
```bash
# Clone the repository
git clone --recursive https://github.com/KevinEppacher/SAGE.git

cd docker && docker compose up -d explorer_container exploitation_container detection_container lavis_container isaac_sim
```
A tmux session is prepared.
```bash
cd path/to/SAGE/docker && ./start_sage_tmux.sh
---
```

## Model Weights

The following pretrained weights are required but are not included in this repository.

1. Install the SEEM weights for the exploitation container:
```bash
cd path/to/SAGE/exploitation_ws/src/openfusion_ros/openfusion_ros/openfusion_ros/zoo/xdecoder_seem/checkpoints
wget https://huggingface.co/xdecoder/SEEM/resolve/main/seem_focall_v1.pt
```

2. Install the YOLO-E weights for the detection container:
```bash
cd path/to/SAGE/detection_ws/src/yolo_ros/yolo_ros/models/yoloe/
wget https://github.com/ultralytics/assets/releases/download/v8.4.0/yoloe-11m-seg.pt
```

## Usage

1. Build the workspaces and launch the nodes in the respective containers:

Launch the explorer container with the navigation, mapping and fusion graph nodes.
```bash
docker exec -it ros2_explorer_container bash
colcon build --symlink-install && source install/setup.bash
ros2 launch sage_commander exploitation_ws.launch.py
```

Launch the BLIP2 module.
```bash
docker exec -it ros2_lavis_container bash
colcon build --symlink-install && source install/setup.bash
ros2 launch blip2_ros blip2_ros.launch.py
```
Launch the detection module.
```bash
docker exec -it ros2_detection_container bash
colcon build --symlink-install && source install/setup.bash
ros2 launch sage_commander detection_ws.launch.py
```

Launch the semantic mapping module.
```bash
docker exec -it ros2_exploitation_container bash
colcon build --symlink-install && source install/setup.bash
ros2 launch sage_commander exploitation_ws.launch.py
```

2. Launch Behavior Tree action server:
```bash
docker exec -it ros2_explorer_container bash
colcon build --symlink-install && source install/setup.bash
ros2 launch sage_behaviour_tree sage_bt_action_server.launch.py
```

3. Launch the Isaac Sim environment in the isaac_sim container:
```bash
docker exec -it isaac_sim_container bash
./runheadless.sh
```

```bash
# On host machine, launch the the Isaac Sim Web Rtc client
cd path/to/isaacsim-webrtc-install/ && ./isaacsim-webrtc-streaming-client-1.0.6-linux-x64.AppImage
```

Within Isaac Sim, load a Habitat Sim environment and start the simulation with the CarterV1 robot.

4. Execute exploration and mapping:
```bash
docker exec -it ros2_explorer_container bash
source install/setup.bash

# Check if all necessary nodes, topics, and services are running
ros2 service call /sage_behaviour_tree/startup_check sage_bt_msgs/srv/StartupCheck

# Send a goal to the behavior tree action server to execute the prompt
ros2 action send_goal /sage_behaviour_tree/execute_prompt sage_bt_msgs/action/ExecutePrompt \
"{ \
  prompt: 'flowerpot', \
  zero_shot_prompt: 'A door leading to a flowerpot', \
  experiment_id: 'EVAL01', \
  save_directory: '/app/test.png', \
  timeout: 60.0, \
  detection_threshold_initial: 0.05, \
  detection_threshold_final: 0.1 \
}" \
--feedback

# To force exit the behavior tree execution (if needed)
ros2 service call /sage_behaviour_tree/force_exit std_srvs/srv/Empty
```

| Parameter | Description |
|---|---|
| `prompt` | Target object or concept to search for. |
| `zero_shot_prompt` | Text prompt used by the zero-shot detector. |
| `experiment_id` | Identifier used for logging and saving results. |
| `save_directory` | Output path for saved results. |
| `timeout` | Maximum execution time in minutes. |
| `detection_threshold_initial` | Initial confidence threshold for detection. |
| `detection_threshold_final` | Final confidence threshold after adjustment. |

## Reproducing Thesis Results

Instructions for reproducing the experiments from the thesis are provided in:
TODO:
- [Experiment setup](./docs/experiments.md)
- [Evaluation scripts](./experiments/README.md)
- [Configuration files](./experiments/configs/)

## Prerequisites

- NVIDIA GPU with CUDA support (for running the containers with GPU acceleration).
- Docker installed on the linux host machine.
- Sufficient disk space for the Docker images and containers.
- Min. 12 GB of RAM for running Isaac Sim + ROS 2 containers simultaneously (this does not include the semantic mapping module, which can consume additional memory depending on the size of the environment and the number of objects detected).

The system was developed and tested with the following setup:

| Component | Version / Requirement |
|---|---|
| OS | Ubuntu 22.04.5 LTS |
| ROS 2 | Humble |
| Docker | 29.4.0 (with Compose plugin) |
| GPU | NVIDIA GeForce RTX 4090 (24 GB VRAM) |
| NVIDIA Driver | 580.126.09 |
| CUDA | 12.1.1 & 11.7.1 |
| RAM | 64 GB |

## Examples

![](./docs/searching_a_bed.mp4)

## Thesis Context

This repository contains the implementation developed for my master thesis:

📄 [Master Thesis PDF](./docs/thesis.pdf)

**Title:** Semantic-Aware Guided Exploration with Persistent Memory  
**Author:** Kevin Eppacher  
**Degree Program:** MSc. - Robotics  
**University:** Fachhochschule Technikum Wien  
**Supervisor:** Simon Schwaiger, MSc.  
**Year:** 2026

## Motivation

Autonomous robots often explore unknown environments using geometric information alone, such as occupancy grids and frontier-based navigation. However, many tasks require semantic understanding: for example, finding objects, remembering previously observed regions, or deciding where to search next based on language prompts.

SAGE addresses this by integrating semantic perception and persistent memory into a ROS 2 exploration pipeline. The system allows a robot to search for object-level goals, maintain a semantic representation of the environment, and use vision-language components to support more informed exploration decisions.

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](./LICENSE) file for details.

## Acknowledgements

This work builds on ROS 2, Nav2, NVIDIA Isaac Sim, OpenFusion, YOLO-E, BLIP2/LAVIS, and SEEM. I would also like to thank my supervisor, research group, and university for their support.