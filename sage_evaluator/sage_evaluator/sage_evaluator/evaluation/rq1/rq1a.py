import json
from pathlib import Path
import numpy as np

# ====================== CONFIG ======================
ROOT = Path("/app/src/sage_evaluator/sage_datasets/matterport_isaac")
RQ = "RQ1"
EXCLUDE_SCENES = {"example_scene"}
# ====================================================

def load_metrics(metrics_file: Path):
    with open(metrics_file, "r") as f:
        data = json.load(f)

    sr_vals = []
    spl_vals = []

    for entry in data.values():
        if "SR" in entry and "SPL" in entry:
            sr_vals.append(entry["SR"])
            spl_vals.append(entry["SPL"])

    return sr_vals, spl_vals


# ====================== DATA COLLECTION ======================
all_sr = []
all_spl = []

total_scenes = 0
total_positions = 0
total_episodes = 0
total_evaluations = 0

for scene in ROOT.iterdir():
    if not scene.is_dir() or scene.name in EXCLUDE_SCENES:
        continue

    rq_root = scene / "episodes" / RQ
    if not rq_root.exists():
        continue

    total_scenes += 1

    for pos_dir in rq_root.iterdir():
        if not pos_dir.is_dir() or not pos_dir.name.startswith("POS"):
            continue

        total_positions += 1

        for episode_dir in pos_dir.iterdir():
            if not episode_dir.is_dir() or not episode_dir.name.startswith("E"):
                continue

            metrics_file = episode_dir / "metrics.json"
            if not metrics_file.exists():
                continue

            total_episodes += 1

            sr_vals, spl_vals = load_metrics(metrics_file)
            all_sr.extend(sr_vals)
            all_spl.extend(spl_vals)
            total_evaluations += len(sr_vals)

# ====================== SAFETY CHECK ======================
if not all_sr:
    raise RuntimeError("No RQ1 metrics found. Check directory structure.")

# ====================== STATISTICS ======================
all_sr = np.array(all_sr)
all_spl = np.array(all_spl)
sr_mean = np.mean(all_sr)
sr_median = np.median(all_sr)
sr_q1 = np.percentile(all_sr, 25)
sr_q3 = np.percentile(all_sr, 75)
sr_var = np.var(all_sr)

spl_mean = np.mean(all_spl)
spl_median = np.median(all_spl)
spl_q1 = np.percentile(all_spl, 25)
spl_q3 = np.percentile(all_spl, 75)
spl_var = np.var(all_spl)

# ====================== PRINT RESULTS ======================
print("\n=== RQ1: Baseline Navigation Performance ===")
print(f"Scenes evaluated        : {total_scenes}")
print(f"Start positions         : {total_positions}")
print(f"Episodes evaluated      : {total_episodes}")
print(f"Total evaluations       : {total_evaluations}")

print("\n--- Success Rate (SR) ---")
print(f"Median SR               : {sr_median:.3f}")
print(f"Mean SR                 : {sr_mean:.3f}")
print(f"IQR (25–75%)            : [{sr_q1:.3f}, {sr_q3:.3f}]")
print(f"Variance SR             : {sr_var:.5f}")

print("\n--- Success weighted Path Length (SPL) ---")
print(f"Median SPL              : {spl_median:.3f}")
print(f"Mean SPL                : {spl_mean:.3f}")
print(f"IQR (25–75%)            : [{spl_q1:.3f}, {spl_q3:.3f}]")
print(f"Variance SPL            : {spl_var:.5f}")
