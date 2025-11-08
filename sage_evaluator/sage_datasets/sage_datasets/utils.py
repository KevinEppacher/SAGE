import os
import glob

# Root where your dataset resides
DATASET_ROOT = "/app/src/sage_evaluator/sage_datasets/matterport_isaac"


def get_annotations_dir(scene: str, version: str) -> str:
    """Return the full path to the specified annotation version directory."""
    ann_dir = os.path.join(DATASET_ROOT, scene, "annotations", version)
    if not os.path.isdir(ann_dir):
        raise FileNotFoundError(f"Annotation directory does not exist: {ann_dir}")
    return ann_dir


def find_latest_file(directory: str, pattern: str) -> str:
    """Return the latest file matching a pattern (based on modification time)."""
    files = glob.glob(os.path.join(directory, pattern))
    if not files:
        raise FileNotFoundError(f"No files matching pattern '{pattern}' in {directory}")
    return max(files, key=os.path.getmtime)


def get_map_path(scene: str, version: str) -> str:
    """Return the path to the SLAM map YAML file."""
    ann_dir = get_annotations_dir(scene, version)
    return find_latest_file(ann_dir, "slam_map_*.yaml")


def get_map_image_path(scene: str, version: str) -> str:
    """Return the corresponding PGM map image path."""
    ann_dir = get_annotations_dir(scene, version)
    return find_latest_file(ann_dir, "slam_map_*.pgm")


def get_pointcloud_path(scene: str, version: str) -> str:
    """Return the semantic pointcloud (.ply) path."""
    ann_dir = get_annotations_dir(scene, version)
    return find_latest_file(ann_dir, "semantic_*.ply")


def get_start_pose_path(scene: str, version: str) -> str:
    """Return the robot_start_pose.json path."""
    ann_dir = get_annotations_dir(scene, version)
    pose_path = os.path.join(ann_dir, "robot_start_pose.json")
    if not os.path.exists(pose_path):
        raise FileNotFoundError(f"robot_start_pose.json not found in {ann_dir}")
    return pose_path


def get_output_dir(scene: str, subfolder: str = "results") -> str:
    """Return the path where generated outputs (images, logs, etc.) should be saved."""
    path = os.path.join(DATASET_ROOT, scene, subfolder)
    os.makedirs(path, exist_ok=True)
    return path

def get_semantic_pcl_classes_path(scene: str, version: str) -> str:
    """Return the path to the semantic pointcloud classes JSON file."""
    ann_dir = get_annotations_dir(scene, version)
    return find_latest_file(ann_dir, "semantic_*.json")

class DatasetManager:
    """Convenient interface for accessing dataset paths by scene and version."""

    def __init__(self, scene: str, version: str):
        self.scene = scene
        self.version = version
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

    def output_dir(self, subfolder: str = "results") -> str:
        return get_output_dir(self.scene, subfolder)

    def __repr__(self):
        return f"<DatasetManager scene='{self.scene}' version='{self.version}' annotations='{self.annotations_dir}'>"
