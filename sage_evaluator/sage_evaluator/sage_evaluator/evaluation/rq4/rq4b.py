import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from collections import Counter

# =====================================================
# CONFIG
# =====================================================
ROOT = Path("/app/src/sage_evaluator/sage_datasets/matterport_isaac")

# Discrete operating points (main paper)
DISCRETE_THRESHOLDS = [0.5, 0.6, 0.8]

# Continuous sweep (robustness / appendix)
SWEEP_THRESHOLDS = np.linspace(0.05, 1.0, 96)  # avoid degenerate τ≈0

FUSION_VARIANTS = {
    "det_only": lambda m: m.get("confidence_detection", 0.0),
    "det_map": lambda m: m.get("confidence_detection", 0.0)
                         + m.get("confidence_vlm", 0.0),
    "det_mem": lambda m: m.get("confidence_detection", 0.0)
                         + m.get("confidence_memory", 0.0),
    "full": lambda m: m.get("confidence_total", 0.0),
}

# =====================================================
# HELPERS
# =====================================================
def load_json(path: Path):
    with open(path, "r") as f:
        return json.load(f)


def classify_detection(meta: dict, score: float, threshold: float):
    failure = meta.get("failure_reason") or meta.get("failure_mode")
    sr = meta.get("SR", 0.0)

    predicted_positive = score >= threshold

    # 1. Active false decision
    if failure == "wrong_object":
        return "FP" if predicted_positive else "TN"

    # 2. Missed decision although observable
    if failure == "ignored":
        return "FN"

    # 3. Successful detection & execution
    if sr == 1.0:
        return "TP" if predicted_positive else "FN"

    # 4. Out-of-scope failures (not detection-related)
    if failure in {"not_seen", "no_motion", "seen_but_far"}:
        return "TN"   # or better: EXCLUDE

    return "TN"



def safe_div(num, denom):
    return num / denom if denom > 0 else 0.0


def compute_metrics(conf):
    TP = conf.get("TP", 0)
    FP = conf.get("FP", 0)
    FN = conf.get("FN", 0)
    TN = conf.get("TN", 0)

    precision = safe_div(TP, TP + FP)
    recall = safe_div(TP, TP + FN)
    f1 = safe_div(2 * precision * recall, precision + recall)
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

# =====================================================
# DATA COLLECTION
# =====================================================
def iterate_episodes():
    for scene in ROOT.iterdir():
        if not scene.is_dir():
            continue

        episodes_dir = scene / "episodes"
        if not episodes_dir.exists():
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

                    metrics_file = episode / "metrics.json"
                    detections = episode / "detections"

                    if not metrics_file.exists() or not detections.exists():
                        continue

                    yield episode, load_json(metrics_file), detections


def collect_discrete_confusions():
    results = {
        v: {t: Counter() for t in DISCRETE_THRESHOLDS}
        for v in FUSION_VARIANTS
    }

    for _, episode_meta, detections in iterate_episodes():
        for obj_dir in detections.iterdir():
            if not obj_dir.is_dir():
                continue

            meta = episode_meta.get(obj_dir.name)
            if meta is None:
                continue

            sr_file = obj_dir / "sr.json"
            if not sr_file.exists():
                continue

            score_meta = load_json(sr_file)

            for variant, fuse_fn in FUSION_VARIANTS.items():
                score = fuse_fn(score_meta)

                for t in DISCRETE_THRESHOLDS:
                    label = classify_detection(meta, score, t)
                    results[variant][t][label] += 1

    return results


def collect_sweep_confusions(variants):
    results = {
        v: {t: Counter() for t in SWEEP_THRESHOLDS}
        for v in variants
    }

    for _, episode_meta, detections in iterate_episodes():
        for obj_dir in detections.iterdir():
            if not obj_dir.is_dir():
                continue

            meta = episode_meta.get(obj_dir.name)
            if meta is None:
                continue

            sr_file = obj_dir / "sr.json"
            if not sr_file.exists():
                continue

            score_meta = load_json(sr_file)

            for variant in variants:
                score = FUSION_VARIANTS[variant](score_meta)

                for t in SWEEP_THRESHOLDS:
                    label = classify_detection(meta, score, t)
                    results[variant][t][label] += 1

    return results

# =====================================================
# VISUALIZATION
# =====================================================
def plot_confusion_matrix(conf, title):
    TP = conf.get("TP", 0)
    FP = conf.get("FP", 0)
    FN = conf.get("FN", 0)
    TN = conf.get("TN", 0)

    cm = np.array([[TP, FP],
                   [FN, TN]])

    fig, ax = plt.subplots()
    im = ax.imshow(cm, cmap="inferno")

    ax.set_xticks([0, 1])
    ax.set_yticks([0, 1])
    ax.set_xticklabels(["Predicted Positive", "Predicted Negative"])
    ax.set_yticklabels(["Actual Positive", "Actual Negative"])

    max_val = cm.max()
    for i in range(2):
        for j in range(2):
            val = cm[i, j]
            color = "black" if val > 0.6 * max_val else "white"
            ax.text(j, i, val, ha="center", va="center",
                    fontsize=16, fontweight="bold", color=color)

    ax.set_title(title)
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    plt.tight_layout()
    plt.show()


def plot_precision_recall_sweep(sweep_confusions):
    fig, ax = plt.subplots()
    cmap = plt.get_cmap("inferno")

    colors = {
        "det_only": cmap(0.35),
        "full": cmap(0.75),
    }

    for variant, threshold_data in sweep_confusions.items():
        points = []

        for t in SWEEP_THRESHOLDS:
            m = compute_metrics(threshold_data[t])
            if m["TP"] == 0:
                continue
            points.append((m["recall"], m["precision"]))

        if not points:
            continue

        points.sort()
        recalls, precisions = zip(*points)

        ax.plot(recalls, precisions,
                linewidth=3.0,
                color=colors[variant],
                label=variant)

    ax.set_xlabel("Recall")
    ax.set_ylabel("Precision")
    ax.set_title("Precision–Recall Curve (Full Threshold Sweep)")
    ax.grid(True)
    ax.legend(title="Fusion Strategy")

    plt.tight_layout()
    plt.show()

# =====================================================
# MAIN
# =====================================================
if __name__ == "__main__":

    # --- Discrete evaluation ---
    discrete = collect_discrete_confusions()

    for variant, thresholds in discrete.items():
        print(f"\n=== Fusion Variant: {variant} ===")
        for t, conf in thresholds.items():
            m = compute_metrics(conf)
            print(
                f"τ={t:.1f} | "
                f"P={m['precision']:.3f} | "
                f"R={m['recall']:.3f} | "
                f"F1={m['f1']:.3f} | "
                f"FPR={m['fpr']:.3f} | "
                f"TP={m['TP']} FP={m['FP']} FN={m['FN']} TN={m['TN']}"
            )

    # --- Confusion matrices at τ=0.8 ---
    plot_confusion_matrix(
        discrete["det_only"][0.8],
        "Confusion Matrix – Detection Only (τ = 0.8)"
    )

    plot_confusion_matrix(
        discrete["full"][0.8],
        "Confusion Matrix – Full Multi-Source Fusion (τ = 0.8)"
    )

    # --- Full PR sweep ---
    sweep = collect_sweep_confusions(["det_only", "full"])
    plot_precision_recall_sweep(sweep)
