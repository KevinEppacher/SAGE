import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import re

# ====================== CONFIG ======================
ROOT = Path("/app/src/sage_evaluator/sage_datasets/matterport_isaac")
RQ = "RQ3"

EXCLUDE_SCENES = {
    "example_scene",
    "00848-ziup5kvtCCR",
}
# ===================================================

TOPK_PATTERN = re.compile(r"TOPK_(\d+)_EXP_(\d+)")

def parse_topk_and_exp(exp_dir: Path):
    """
    Parses directory name like:
    TOPK_20_EXP_60 -> (20, "EXP_60")
    """
    m = TOPK_PATTERN.match(exp_dir.name)
    if not m:
        return None
    return int(m.group(1)), f"EXP_{m.group(2)}"


def load_metrics(metrics_file: Path):
    with open(metrics_file, "r") as f:
        data = json.load(f)

    sr_vals, spl_vals = [], []

    for entry in data.values():
        if "SR" in entry and "SPL" in entry:
            sr_vals.append(entry["SR"])
            spl_vals.append(entry["SPL"])

    if not sr_vals:
        return None

    return np.mean(sr_vals), np.mean(spl_vals)


def aggregate(data_dict):
    xs, med, q1, q3 = [], [], [], []
    for k in sorted(data_dict.keys()):
        vals = np.array(data_dict[k])
        xs.append(k)
        med.append(np.median(vals))
        q1.append(np.percentile(vals, 25))
        q3.append(np.percentile(vals, 75))
    return np.array(xs), np.array(med), np.array(q1), np.array(q3)


# ===================================================
# DATA COLLECTION
# scene -> mode -> topk -> list
# ===================================================
sr_scene = {}
spl_scene = {}

print("Starting data collection...")

for scene in ROOT.iterdir():
    if not scene.is_dir() or scene.name in EXCLUDE_SCENES:
        continue

    rq_root = scene / "episodes" / RQ
    if not rq_root.exists():
        continue

    print(f"Processing scene: {scene.name}")

    for episode_dir in rq_root.iterdir():
        if not episode_dir.is_dir():
            continue

        for exp in episode_dir.iterdir():
            parsed = parse_topk_and_exp(exp)
            if parsed is None:
                continue

            topk, mode = parsed
            if mode not in {"EXP_60", "EXP_0"}:
                continue

            metrics_file = exp / "metrics.json"
            if not metrics_file.exists():
                continue

            result = load_metrics(metrics_file)
            if result is None:
                continue

            sr_val, spl_val = result

            sr_scene.setdefault(scene.name, {}).setdefault(mode, {}).setdefault(topk, []).append(sr_val)
            spl_scene.setdefault(scene.name, {}).setdefault(mode, {}).setdefault(topk, []).append(spl_val)


if not sr_scene:
    raise RuntimeError("No valid data found.")

# ===================================================
# PLOTTING
# ===================================================
def plot_per_scene(data_scene, ylabel, title):
    scenes = sorted(data_scene.keys())
    n = len(scenes)

    cols = 3
    rows = int(np.ceil(n / cols))

    fig, axes = plt.subplots(
        rows, cols,
        figsize=(5 * cols, 4 * rows),
        sharex=True,
        sharey=True
    )

    axes = axes.flatten()

    for ax, scene in zip(axes, scenes):
        scene_data = data_scene[scene]

        if "EXP_60" in scene_data:
            x, med, q1, q3 = aggregate(scene_data["EXP_60"])
            ax.plot(x, med, marker="o", label="EXP_60", color="tab:blue")
            ax.fill_between(x, q1, q3, alpha=0.25, color="tab:blue")

        if "EXP_0" in scene_data:
            x, med, q1, q3 = aggregate(scene_data["EXP_0"])
            ax.plot(x, med, marker="s", label="EXP_0", color="tab:orange")
            ax.fill_between(x, q1, q3, alpha=0.25, color="tab:orange")

        ax.set_title(scene, fontsize=14)
        ax.grid(True)

    for ax in axes[len(scenes):]:
        ax.axis("off")

    fig.supxlabel("Semantic Retrieval Granularity (TOPK)", fontsize=18)
    fig.supylabel(ylabel, fontsize=18)
    fig.suptitle(title, fontsize=20)

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=2, fontsize=14)

    fig.tight_layout(rect=[0, 0, 1, 0.92])
    plt.show()


# ====================== RUN PLOTS ======================
plot_per_scene(
    sr_scene,
    ylabel="Success Rate (SR)",
    title="SR vs Semantic Map Granularity per Scene (RQ3)"
)

plot_per_scene(
    spl_scene,
    ylabel="Success weighted Path Length (SPL)",
    title="SPL vs Semantic Map Granularity per Scene (RQ3)"
)
