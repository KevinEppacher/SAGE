import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

# ====================== CONFIG ======================
ROOT = Path("/app/src/sage_evaluator/sage_datasets/matterport_isaac")
RQ = "RQ2"
EXCLUDE_SCENES = {"example_scene"}
# ====================================================

def parse_hyperparam(exp_dir: Path) -> float:
    # EXP_MEM_40_60 -> exploitation weight = 0.6
    parts = exp_dir.name.split("_")
    explore = float(parts[2])
    exploit = float(parts[3])
    return exploit / (explore + exploit)

def load_metrics(metrics_file: Path):
    with open(metrics_file, "r") as f:
        data = json.load(f)

    sr_vals = []
    spl_vals = []

    for entry in data.values():
        if "SR" in entry and "SPL" in entry:
            sr_vals.append(entry["SR"])
            spl_vals.append(entry["SPL"])

    if not sr_vals:
        return None

    # per-episode aggregation over prompts
    return np.mean(sr_vals), np.mean(spl_vals)

# hyperparameter -> list of values (scene × episode)
sr_per_h = {}
spl_per_h = {}

# ====================== DATA COLLECTION ======================
for scene in ROOT.iterdir():
    if not scene.is_dir() or scene.name in EXCLUDE_SCENES:
        continue

    rq_root = scene / "episodes" / RQ
    if not rq_root.exists():
        continue

    for episode_dir in rq_root.iterdir():  # E001, E002, ...
        if not episode_dir.is_dir() or not episode_dir.name.startswith("E"):
            continue

        for exp in episode_dir.iterdir():
            if not exp.name.startswith("EXP_MEM_"):
                continue

            metrics_file = exp / "metrics.json"
            if not metrics_file.exists():
                continue

            h = parse_hyperparam(exp)
            result = load_metrics(metrics_file)
            if result is None:
                continue

            sr, spl = result
            sr_per_h.setdefault(h, []).append(sr)
            spl_per_h.setdefault(h, []).append(spl)

# ====================== SAFETY CHECK ======================
if not sr_per_h:
    raise RuntimeError("No metrics found. Check directory structure.")

# ====================== SORT ======================
x = np.array(sorted(sr_per_h.keys()))

# ====================== STATISTICS (MEDIAN + IQR) ======================
median_sr, q1_sr, q3_sr = [], [], []
median_spl, q1_spl, q3_spl = [], [], []

for h in x:
    sr_vals = np.array(sr_per_h[h])
    spl_vals = np.array(spl_per_h[h])

    median_sr.append(np.median(sr_vals))
    q1_sr.append(np.percentile(sr_vals, 25))
    q3_sr.append(np.percentile(sr_vals, 75))

    median_spl.append(np.median(spl_vals))
    q1_spl.append(np.percentile(spl_vals, 25))
    q3_spl.append(np.percentile(spl_vals, 75))

median_sr = np.array(median_sr)
q1_sr = np.array(q1_sr)
q3_sr = np.array(q3_sr)

median_spl = np.array(median_spl)
q1_spl = np.array(q1_spl)
q3_spl = np.array(q3_spl)

fontsize = 14
axis_fontsize = 18

# ====================== PLOT: SR ======================
plt.figure()
plt.plot(x, median_sr, marker="o", label="Median SR")
plt.fill_between(x, q1_sr, q3_sr, alpha=0.25, label="IQR (25–75%)")
plt.xlabel("Exploitation weight", fontsize=fontsize)
plt.ylabel("Success Rate (SR)", fontsize=fontsize)
plt.title("SR vs Exploration–Exploitation Trade-off", fontsize=fontsize)
plt.legend(fontsize=fontsize)
plt.grid(True)
plt.xticks(fontsize=axis_fontsize)
plt.yticks(fontsize=axis_fontsize)
plt.tight_layout()

# ====================== PLOT: SPL ======================
plt.figure()
plt.plot(x, median_spl, marker="o", label="Median SPL")
plt.fill_between(x, q1_spl, q3_spl, alpha=0.25, label="IQR (25–75%)")
plt.xlabel("Exploitation weight", fontsize=fontsize)
plt.ylabel("Success weighted Path Length (SPL)", fontsize=fontsize)
plt.title("SPL vs Exploration–Exploitation Trade-off", fontsize=fontsize)
plt.legend(fontsize=fontsize)
plt.grid(True)
plt.xticks(fontsize=axis_fontsize)
plt.yticks(fontsize=axis_fontsize)
plt.tight_layout()

plt.show()