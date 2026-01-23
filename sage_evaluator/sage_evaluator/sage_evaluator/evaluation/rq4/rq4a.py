import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from collections import Counter

# =====================================================
# CONFIG
# =====================================================
ROOT = Path("/app/src/sage_evaluator/sage_datasets/matterport_isaac")

DISCRETE_THRESHOLDS = [0.5, 0.6, 0.8]
SWEEP_THRESHOLDS = np.linspace(0.05, 1.0, 96)

# For snapshot dataset, we fuse per-node fields and then take max over nodes.
# - detection_confidence: YOLOE only
# - vlm_confidence: map/VLM source
# - memory_confidence: memory source
# - score: your already-fused final score (full)
FUSION_VARIANTS = {
    "det_only": lambda n: float(n.get("detection_confidence", 0.0)),
    "det_map":  lambda n: float(n.get("detection_confidence", 0.0)) + float(n.get("vlm_confidence", 0.0)),
    "det_mem":  lambda n: float(n.get("detection_confidence", 0.0)) + float(n.get("memory_confidence", 0.0)),
    "full":     lambda n: float(n.get("score", 0.0)),
}

# =====================================================
# HELPERS
# =====================================================
def load_json(path: Path):
    with open(path, "r") as f:
        return json.load(f)


def iterate_snapshots():
    """
    Yields (meta, graph_nodes) for every snapshot directory.
    Expected structure:
    <scene>/episodes/<RQx>/<snapshot_dir>/{annotation.json,graph_nodes.json}
    """
    for scene in ROOT.iterdir():
        if not scene.is_dir():
            continue

        episodes_dir = scene / "episodes"
        if not episodes_dir.exists():
            continue

        for rq in episodes_dir.iterdir():
            if not rq.is_dir():
                continue

            for snap in rq.iterdir():
                if not snap.is_dir():
                    continue

                meta_file = snap / "annotation.json"
                graph_file = snap / "graph_nodes.json"

                if not meta_file.exists() or not graph_file.exists():
                    continue

                yield load_json(meta_file), load_json(graph_file)


def per_snapshot_score(graph_nodes: dict, per_node_score_fn) -> float:
    nodes = graph_nodes.get("nodes", [])
    if not nodes:
        return 0.0
    return max(per_node_score_fn(n) for n in nodes)

def classify_from_annotation(meta: dict, predicted_positive: bool) -> str:
    """
    New snapshot dataset:
    meta["target_present"] ∈ {True, False}
    """
    gt_positive = bool(meta.get("target_present", False))

    if gt_positive and predicted_positive:
        return "TP"
    if gt_positive and not predicted_positive:
        return "FN"
    if (not gt_positive) and predicted_positive:
        return "FP"
    return "TN"

def safe_div(num, denom):
    return num / denom if denom > 0 else 0.0


def compute_metrics(conf: Counter):
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
def collect_discrete_confusions():
    results = {
        v: {t: Counter() for t in DISCRETE_THRESHOLDS}
        for v in FUSION_VARIANTS
    }

    for meta, graph in iterate_snapshots():
        for variant, per_node_fn in FUSION_VARIANTS.items():
            score = per_snapshot_score(graph, per_node_fn)

            for t in DISCRETE_THRESHOLDS:
                predicted_positive = score >= t
                lab = classify_from_annotation(meta, predicted_positive)
                results[variant][t][lab] += 1

    return results

def collect_sweep_confusions(variants):
    results = {
        v: {t: Counter() for t in SWEEP_THRESHOLDS}
        for v in variants
    }

    for meta, graph in iterate_snapshots():
        for variant in variants:
            score = per_snapshot_score(graph, FUSION_VARIANTS[variant])

            for t in SWEEP_THRESHOLDS:
                predicted_positive = score >= t
                lab = classify_from_annotation(meta, predicted_positive)
                results[variant][t][lab] += 1

    return results

# =====================================================
# VISUALIZATION
# =====================================================
def plot_confusion_matrix(conf: Counter, title: str, fontsize=16, axis_fontsize=14):
    TP = conf.get("TP", 0)
    FP = conf.get("FP", 0)
    FN = conf.get("FN", 0)
    TN = conf.get("TN", 0)

    cm = np.array([
        [TP, FP],
        [FN, TN],
    ])

    fig, ax = plt.subplots(figsize=(5, 4))
    im = ax.imshow(cm, cmap="inferno")

    ax.set_xticks([0, 1])
    ax.set_yticks([0, 1])

    ax.set_xticklabels(["Actual Positive", "Actual Negative"], fontsize=axis_fontsize)
    ax.set_yticklabels(["Predicted\nPositive", "Predicted\nNegative"], fontsize=axis_fontsize)

    max_val = cm.max() if cm.size else 0
    for i in range(2):
        for j in range(2):
            val = int(cm[i, j])
            color = "black" if (max_val > 0 and val > 0.6 * max_val) else "white"
            ax.text(
                j, i, f"{val}",
                ha="center", va="center",
                fontsize=fontsize, fontweight="bold",
                color=color
            )

    ax.set_title(title, fontsize=fontsize)
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

    variant_labels = {
        "det_only": "Detection only",
        "full": "Multi-source fusion",
    }

    fontsize = 18
    axis_fontsize = 20

    for variant, threshold_data in sweep_confusions.items():
        points = []
        for t in SWEEP_THRESHOLDS:
            m = compute_metrics(threshold_data[t])
            if m["TP"] == 0 and m["FP"] == 0:
                continue
            points.append((m["recall"], m["precision"]))

        if not points:
            continue

        points.sort()
        recalls, precisions = zip(*points)

        ax.plot(
            recalls,
            precisions,
            linewidth=3.0,
            color=colors.get(variant, None),
            label=variant_labels.get(variant, variant),
        )

    ax.tick_params(axis="both", which="major", labelsize=axis_fontsize)
    ax.set_xlabel("Recall", fontsize=axis_fontsize)
    ax.set_ylabel("Precision", fontsize=axis_fontsize)
    ax.set_title("Precision–Recall Curve (Full Threshold Sweep)", fontsize=fontsize)
    ax.grid(True)
    ax.legend(title="Fusion Strategy", fontsize=14)

    plt.tight_layout()
    plt.show()

# =====================================================
# MAIN
# =====================================================
if __name__ == "__main__":

    # --- Discrete evaluation (table output identical style) ---
    discrete = collect_discrete_confusions()

    for variant, thresholds in discrete.items():
        print(f"\n=== Fusion Variant: {variant} ===")
        for t in DISCRETE_THRESHOLDS:
            conf = thresholds[t]
            m = compute_metrics(conf)
            print(
                f"τ={t:.1f} | "
                f"P={m['precision']:.3f} | "
                f"R={m['recall']:.3f} | "
                f"F1={m['f1']:.3f} | "
                f"FPR={m['fpr']:.3f} | "
                f"TP={m['TP']} FP={m['FP']} FN={m['FN']} TN={m['TN']}"
            )

    # --- Confusion matrices at τ=0.8 (both det_only and full) ---
    plot_confusion_matrix(
        discrete["det_only"][0.8],
        "Confusion Matrix:\n Detection Only (τ = 0.8)"
    )

    plot_confusion_matrix(
        discrete["full"][0.8],
        "Confusion Matrix:\n Full Multi-Source Fusion (τ = 0.8)"
    )

    # --- Full PR sweep (det_only vs full) ---
    sweep = collect_sweep_confusions(["det_only", "full"])
    plot_precision_recall_sweep(sweep)
