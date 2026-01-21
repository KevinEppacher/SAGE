import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from collections import Counter

YELLOW = "\033[33m"
RESET = "\033[0m"

ROOT = Path("/app/src/sage_evaluator/sage_datasets/matterport_isaac")
CONF_THRESHOLD = 0.8

def load_json(path: Path):
    with open(path, "r") as f:
        return json.load(f)

def classify_detection(meta_entry: dict):
    failure = meta_entry.get("failure_reason") or meta_entry.get("failure_mode")
    sr = meta_entry.get("SR", 0.0)

    if failure == "wrong_object":
        return "FP"

    if failure in {"ignored", "not_seen", "no_motion", "seen_but_far"}:
        return "FN"

    if sr == 1.0:
        return "TP"

    # Conservative fallback
    return "FN"

def collect_confusion():
    counts = Counter()

    for scene in ROOT.iterdir():
        if not scene.is_dir():
            continue

        episodes_dir = scene / "episodes"
        if not episodes_dir.exists() or not episodes_dir.is_dir():
            continue

        for rq in episodes_dir.iterdir():
            if not rq.is_dir():
                continue

            for pos in rq.iterdir():
                if not pos.is_dir():
                    continue

                for episode in pos.iterdir():
                    if not episode.is_dir():
                        continue

                    detections = episode / "detections"
                    if not detections.exists() or not detections.is_dir():
                        continue

                    metrics_file = episode / "metrics.json"
                    episode_meta = load_json(metrics_file) if metrics_file.exists() else {}

                    for obj_dir in detections.iterdir():
                        if not obj_dir.is_dir():
                            continue

                        meta = episode_meta.get(obj_dir.name)
                        if meta is None:
                            continue

                        # --- WARNING CHECK ---
                        sr = meta.get("SR", None)
                        failure = meta.get("failure_reason") or meta.get("failure_mode")

                        if sr == 0.0 and failure is None:
                            print(
                                YELLOW
                                + "[WARNING] SR == 0.0 but no failure_reason set:\n"
                                f"  Scene:    {scene.name}\n"
                                f"  RQ:       {rq.name}\n"
                                f"  Position: {pos.name}\n"
                                f"  Episode:  {episode.name}\n"
                                f"  Object:   {obj_dir.name}\n"
                                f"  Path:     {episode}\n"
                                + RESET
                            )


                        label = classify_detection(meta)
                        counts[label] += 1

    return counts


conf = collect_confusion()
print(conf)

def build_confusion_matrix(conf):
    TP = conf.get("TP", 0)
    FP = conf.get("FP", 0)
    FN = conf.get("FN", 0)
    TN = 0  # structurally undefined

    return np.array([
        [TP, FP],
        [FN, TN]
    ])

cm = build_confusion_matrix(conf)

def plot_confusion_matrix(cm):
    fig, ax = plt.subplots()
    im = ax.imshow(cm)

    ax.set_xticks([0, 1])
    ax.set_yticks([0, 1])
    ax.set_xticklabels(["Predicted Positive", "Predicted Negative"])
    ax.set_yticklabels(["Actual Positive", "Actual Negative"])

    for i in range(2):
        for j in range(2):
            ax.text(j, i, cm[i, j],
                    ha="center", va="center")

    ax.set_title("Confusion Matrix (Goal-Conditioned Object Search)")
    plt.tight_layout()
    plt.show()

def safe_div(num, denom):
    return num / denom if denom > 0 else 0.0

def compute_metrics(conf):
    TP = conf.get("TP", 0)
    FP = conf.get("FP", 0)
    FN = conf.get("FN", 0)
    TN = conf.get("TN", 0)  # will be 0

    precision = safe_div(TP, TP + FP)
    recall = safe_div(TP, TP + FN)

    f1 = (
        2 * precision * recall / (precision + recall)
        if (precision + recall) > 0 else 0.0
    )

    fpr = safe_div(FP, FP + TN)

    return {
        "TP": TP,
        "FP": FP,
        "FN": FN,
        "TN": TN,
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "fpr": fpr,
    }

metrics = compute_metrics(conf)
for k, v in metrics.items():
    print(f"{k}: {v:.4f}" if isinstance(v, float) else f"{k}: {v}")

plot_confusion_matrix(cm)
