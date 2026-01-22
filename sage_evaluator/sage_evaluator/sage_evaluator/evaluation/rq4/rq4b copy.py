import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from collections import Counter

# ---------------------------
# CONFIG
# ---------------------------
ROOT = Path("/app/src/sage_evaluator/sage_datasets/matterport_isaac")

THRESHOLDS = [0.5, 0.6, 0.8]

FUSION_VARIANTS = {
    "det_only": lambda m: m.get("confidence_detection", 0.0),
    "det_map": lambda m: m.get("confidence_detection", 0.0)
                           + m.get("confidence_vlm", 0.0),
    "det_mem": lambda m: m.get("confidence_detection", 0.0)
                           + m.get("confidence_memory", 0.0),
    "full": lambda m: m.get("confidence_total", 0.0),
}

# ---------------------------
# HELPERS
# ---------------------------
def load_json(path: Path):
    with open(path, "r") as f:
        return json.load(f)


def classify_detection(meta: dict, score: float, threshold: float):
    """
    Maps detection outcome to confusion matrix entry.
    """
    failure = meta.get("failure_reason") or meta.get("failure_mode")

    sr = meta.get("SR", 0.0)

    predicted_positive = score >= threshold

    if failure == "wrong_object":
        return "FP" if predicted_positive else "TN"

    if failure in {"ignored", "not_seen", "no_motion", "seen_but_far"}:
        return "FN" if predicted_positive else "TN"

    if sr == 1.0:
        return "TP" if predicted_positive else "FN"

    return "FN"


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


# ---------------------------
# MAIN COLLECTION
# ---------------------------
def collect_all_confusions():
    results = {
        variant: {t: Counter() for t in THRESHOLDS}
        for variant in FUSION_VARIANTS
    }

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
                    if not metrics_file.exists():
                        continue

                    episode_meta = load_json(metrics_file)

                    detections = episode / "detections"
                    if not detections.exists():
                        continue

                    for obj_dir in detections.iterdir():
                        if not obj_dir.is_dir():
                            continue

                        # ---------- GT ----------
                        meta = episode_meta.get(obj_dir.name)
                        if meta is None:
                            continue

                        # ---------- PREDICTION ----------
                        sr_file = obj_dir / "sr.json"
                        if not sr_file.exists():
                            continue

                        score_meta = load_json(sr_file)

                        for variant, fuse_fn in FUSION_VARIANTS.items():
                            score = fuse_fn(score_meta)

                            for t in THRESHOLDS:
                                label = classify_detection(meta, score, t)
                                results[variant][t][label] += 1

    return results


# ---------------------------
# RUN + PRINT RESULTS
# ---------------------------
all_confusions = collect_all_confusions()

for variant, thresholds in all_confusions.items():
    print(f"\n=== Fusion Variant: {variant} ===")
    for t, conf in thresholds.items():
        metrics = compute_metrics(conf)
        print(
            f"τ={t:.1f} | "
            f"P={metrics['precision']:.3f} | "
            f"R={metrics['recall']:.3f} | "
            f"F1={metrics['f1']:.3f} | "
            f"FPR={metrics['fpr']:.3f} | "
            f"TP={metrics['TP']} FP={metrics['FP']} FN={metrics['FN']} TN={metrics['TN']}"
        )


# ---------------------------
# OPTIONAL: CONFUSION MATRIX PLOT
# ---------------------------
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

            # Choose text color based on relative intensity
            text_color = "black" if val > 0.6 * max_val else "white"

            ax.text(
                j, i, f"{val}",
                ha="center",
                va="center",
                color=text_color,
                fontsize=16,
                fontweight="bold"
            )

    ax.set_title(title)
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

    plt.tight_layout()
    plt.show()


# ---------------------------
# PRECISION–RECALL CURVES
# ---------------------------
def plot_precision_recall(all_confusions):
    fig, ax = plt.subplots()

    variants_to_plot = ["det_only", "full"]
    cmap = plt.get_cmap("inferno")
    colors = {
        "det_only": cmap(0.35),
        "full": cmap(0.75),
    }

    for variant in variants_to_plot:
        thresholds = all_confusions[variant]
        recalls = []
        precisions = []

        for t in sorted(THRESHOLDS):
            conf = thresholds[t]
            metrics = compute_metrics(conf)
            recalls.append(metrics["recall"])
            precisions.append(metrics["precision"])

        ax.plot(
            recalls,
            precisions,
            marker="o",
            linewidth=2.8,
            markersize=7,
            color=colors[variant],
            label=variant
        )

        for r, p, t in zip(recalls, precisions, THRESHOLDS):
            ax.annotate(
                f"τ={t}",
                (r, p),
                textcoords="offset points",
                xytext=(6, 6),
                fontsize=10,
                color=colors[variant]
            )

    # Automatic zoom to relevant region
    all_recalls = []
    all_precisions = []

    for v in variants_to_plot:
        for conf in all_confusions[v].values():
            m = compute_metrics(conf)
            all_recalls.append(m["recall"])
            all_precisions.append(m["precision"])

    ax.set_xlim(min(all_recalls) - 0.02, max(all_recalls) + 0.02)
    ax.set_ylim(min(all_precisions) - 0.02, 1.01)

    ax.set_xlabel("Recall")
    ax.set_ylabel("Precision")
    ax.set_title("Precision–Recall: Detection Only vs Full Fusion")
    ax.grid(True)
    ax.legend(title="Fusion Variant")

    plt.tight_layout()
    plt.show()


# --- Call plot ---
plot_precision_recall(all_confusions)

# Example plot (full fusion, τ=0.8)
plot_confusion_matrix(
    all_confusions["det_only"][0.8],
    "Confusion Matrix – Detection Only (τ = 0.8)"
)

plot_confusion_matrix(
    all_confusions["full"][0.8],
    "Confusion Matrix – Full Multi-Source Fusion (τ = 0.8)"
)
