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
    # EXP_MEM_40_60 -> exploration weight = 0.4
    parts = exp_dir.name.split("_")
    explore = float(parts[2])
    exploit = float(parts[3])
    return explore / (explore + exploit)

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

    return np.mean(sr_vals), np.mean(spl_vals)

# hyperparameter -> list of values
sr_per_h = {}
spl_per_h = {}

# ====================== DATA COLLECTION ======================
for scene in ROOT.iterdir():
    if not scene.is_dir() or scene.name in EXCLUDE_SCENES:
        continue

    rq_root = scene / "episodes" / RQ
    if not rq_root.exists():
        continue

    for episode_dir in rq_root.iterdir():
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

# ====================== COUNTING ======================
total_episodes = 0
multi_object_episodes = 0
total_prompts = 0

for scene in ROOT.iterdir():
    if not scene.is_dir() or scene.name in EXCLUDE_SCENES:
        continue

    rq_root = scene / "episodes" / RQ
    if not rq_root.exists():
        continue

    for episode_dir in rq_root.iterdir():
        if not episode_dir.is_dir() or not episode_dir.name.startswith("E"):
            continue

        # Count episode once
        episode_has_metrics = False
        episode_prompt_count = 0

        for exp in episode_dir.iterdir():
            if not exp.name.startswith("EXP_MEM_"):
                continue

            metrics_file = exp / "metrics.json"
            if not metrics_file.exists():
                continue

            episode_has_metrics = True

            with open(metrics_file, "r") as f:
                data = json.load(f)

            num_prompts = len(data)
            episode_prompt_count += num_prompts
            total_prompts += num_prompts

        if episode_has_metrics:
            total_episodes += 1
            if episode_prompt_count > 1:
                multi_object_episodes += 1

# ====================== PRINT SUMMARY ======================
print("\n=== Dataset Summary (RQ2) ===")
print(f"Total episodes evaluated        : {total_episodes}")
print(f"Multi-object episodes (>1 obj)  : {multi_object_episodes}")
print(f"Total prompts evaluated         : {total_prompts}")


# ====================== SAFETY CHECK ======================
if not sr_per_h:
    raise RuntimeError("No metrics found. Check directory structure.")

# ====================== SORT ======================
x = np.array(sorted(sr_per_h.keys()))

# ====================== STATISTICS ======================
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

# ====================== COLOR SETUP (INFERNO) ======================
cmap = plt.cm.viridis
main_color = cmap(0.6)      # consistent inferno tone
fill_alpha = 0.25

fontsize = 16
axis_fontsize = 20

# ====================== PLOT: SR ======================
plt.figure()
plt.plot(
    x, median_sr,
    marker="o",
    color=main_color,
    label="Median SR"
)
plt.fill_between(
    x, q1_sr, q3_sr,
    color=main_color,
    alpha=fill_alpha,
    label="IQR (25–75%)"
)

plt.xlabel("Exploration weight", fontsize=fontsize)
plt.ylabel("Success Rate (SR)", fontsize=fontsize)
plt.title("SR vs Exploration–Exploitation Trade-off (RQ2)", fontsize=fontsize)
plt.legend(fontsize=fontsize)
plt.grid(True)
plt.xticks(fontsize=axis_fontsize)
plt.yticks(fontsize=axis_fontsize)
plt.tight_layout()

# ====================== PLOT: SPL ======================
plt.figure()
plt.plot(
    x, median_spl,
    marker="o",
    color=main_color,
    label="Median SPL"
)
plt.fill_between(
    x, q1_spl, q3_spl,
    color=main_color,
    alpha=fill_alpha,
    label="IQR (25–75%)"
)

plt.xlabel("Exploration weight", fontsize=fontsize)
plt.ylabel("Success weighted Path Length (SPL)", fontsize=fontsize)
plt.title("SPL vs Exploration–Exploitation Trade-off (RQ2)", fontsize=fontsize)
plt.legend(fontsize=fontsize)
plt.grid(True)
plt.xticks(fontsize=axis_fontsize)
plt.yticks(fontsize=axis_fontsize)
plt.tight_layout()

plt.show()
