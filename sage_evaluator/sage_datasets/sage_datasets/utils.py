import os
import glob
import json
from datetime import datetime

DATASET_ROOT = "/app/src/sage_evaluator/sage_datasets/matterport_isaac"

def get_annotations_dir(scene: str, version: str) -> str:
    ann_dir = os.path.join(DATASET_ROOT, scene, "annotations", version)
    if not os.path.isdir(ann_dir):
        raise FileNotFoundError(f"Annotation directory does not exist: {ann_dir}")
    return ann_dir

def find_latest_file(directory: str, pattern: str) -> str:
    files = glob.glob(os.path.join(directory, pattern))
    if not files:
        raise FileNotFoundError(f"No files matching pattern '{pattern}' in {directory}")
    return max(files, key=os.path.getmtime)

def get_map_path(scene: str, version: str) -> str:
    ann_dir = get_annotations_dir(scene, version)
    return find_latest_file(ann_dir, "slam_map_*.yaml")

def get_map_image_path(scene: str, version: str) -> str:
    ann_dir = get_annotations_dir(scene, version)
    return find_latest_file(ann_dir, "slam_map_*.pgm")

def get_pointcloud_path(scene: str, version: str) -> str:
    ann_dir = get_annotations_dir(scene, version)
    return find_latest_file(ann_dir, "semantic_*.ply")

def get_start_pose_path(scene: str, version: str) -> str:
    ann_dir = get_annotations_dir(scene, version)
    pose_path = os.path.join(ann_dir, "robot_start_pose.json")
    if not os.path.exists(pose_path):
        raise FileNotFoundError(f"robot_start_pose.json not found in {ann_dir}")
    return pose_path

def get_semantic_pcl_classes_path(scene: str, version: str) -> str:
    ann_dir = get_annotations_dir(scene, version)
    return find_latest_file(ann_dir, "semantic_*.json")

# --------------------------------------------------------------------------- #
# Prompt + Episode management
# --------------------------------------------------------------------------- #

def get_prompts_path(scene: str, episode: str) -> str:
    path = os.path.join(DATASET_ROOT, scene, "episodes", episode, "prompts.json")
    if not os.path.exists(path):
        raise FileNotFoundError(f"prompts.json not found for scene {scene}, episode {episode}")
    return path

def load_prompts(scene: str, episode: str) -> dict:
    prompts_path = get_prompts_path(scene, episode)
    with open(prompts_path, "r") as f:
        return json.load(f)

def get_episode_dir(scene: str, episode_id: str) -> str:
    """Return (and create if missing) a stable directory for this episode."""
    episode_dir = os.path.join(DATASET_ROOT, scene, "episodes", episode_id)
    os.makedirs(episode_dir, exist_ok=True)
    return episode_dir

# --------------------------------------------------------------------------- #
# DatasetManager Class
# --------------------------------------------------------------------------- #
class DatasetManager:
    """Convenient interface for accessing dataset paths by scene, version, and episode."""

    def __init__(self, scene: str, version: str, episode: str = None):
        self.scene = scene
        self.version = version
        self.episode = episode
        self.annotations_dir = get_annotations_dir(scene, version)

    def map(self) -> str:
        return get_map_path(self.scene, self.version)

    def map_image(self) -> str:
        return get_map_image_path(self.scene, self.version)

    def pose(self) -> str:
        return get_start_pose_path(self.scene, self.version)

    def pointcloud(self) -> str:
        return get_pointcloud_path(self.scene, self.version)

    def semantic_pcl_classes(self) -> str:
        return get_semantic_pcl_classes_path(self.scene, self.version)

    def prompts(self) -> dict:
        if not self.episode:
            raise ValueError("Episode ID required to load prompts.")
        return load_prompts(self.scene, self.episode)

    def episode_dir(self) -> str:
        if not self.episode:
            raise ValueError("Episode ID required to access episode directory.")
        return get_episode_dir(self.scene, self.episode)

    def __repr__(self):
        return f"<DatasetManager scene='{self.scene}' version='{self.version}' episode='{self.episode}'>"
