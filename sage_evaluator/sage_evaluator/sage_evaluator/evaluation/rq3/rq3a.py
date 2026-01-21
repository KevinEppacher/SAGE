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
    # "00800-TEEsavR23oF",
    # "00813-svBbv1Pavdk",
    # "00814-p53SfW6mjZe",
    # "00824-Dd4bFSTQ8gi",
    # "00848-ziup5kvtCCR",
    "00876-mv2HUxq3B53",
    }
# ====================================================

TOPK_PATTERN = re.compile(r"TOPK_(\d+)_EXP_(\d+)")

def parse_topk_and_exp(exp_dir: Path):
    """
    Parses TOPK_20_EXP_60 → (20, 60)
    """
    m = TOPK_PATTERN.match(exp_dir.name)
    if not m:
        return None
    return int(m.group(1)), int(m.group(2))


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


# exp_mode -> topk -> list of values
# exp_mode ∈ {"EXP_60", "EXP_0"}
sr = {"EXP_60": {}, "EXP_0": {}}
spl = {"EXP_60": {}, "EXP_0": {}}

print("Starting data collection...")
print(f"Root path: {ROOT}")
print(f"Excluding scenes: {EXCLUDE_SCENES}")

# ====================== DATA COLLECTION ======================
for scene in ROOT.iterdir():
    if not scene.is_dir() or scene.name in EXCLUDE_SCENES:
        continue

    rq_root = scene / "episodes" / RQ
    if not rq_root.exists():
        continue

    print(f"Processing scene: {scene.name}")

    for episode_dir in rq_root.iterdir():  # E001, E002, ...
        if not episode_dir.is_dir():
            continue

        for exp in episode_dir.iterdir():
            parsed = parse_topk_and_exp(exp)
            if parsed is None:
                continue

            topk, exp_weight = parsed
            mode = f"EXP_{exp_weight}"

            if mode not in sr:
                continue

            metrics_file = exp / "metrics.json"
            if not metrics_file.exists():
                continue

            result = load_metrics(metrics_file)
            if result is None:
                continue

            sr_val, spl_val = result
            sr[mode].setdefault(topk, []).append(sr_val)
            spl[mode].setdefault(topk, []).append(spl_val)


# ====================== SAFETY CHECK ======================
if not sr["EXP_60"] or not sr["EXP_0"]:
    raise RuntimeError("Missing EXP_60 or EXP_0 data. Check folder structure.")


def aggregate(data_dict):
    xs, med, q1, q3 = [], [], [], []
    for k in sorted(data_dict.keys()):
        vals = np.array(data_dict[k])
        xs.append(k)
        med.append(np.median(vals))
        q1.append(np.percentile(vals, 25))
        q3.append(np.percentile(vals, 75))
    return np.array(xs), np.array(med), np.array(q1), np.array(q3)


# ====================== AGGREGATION ======================
x60, sr60, sr60_q1, sr60_q3 = aggregate(sr["EXP_60"])
x0,  sr0,  sr0_q1,  sr0_q3  = aggregate(sr["EXP_0"])

_, spl60, spl60_q1, spl60_q3 = aggregate(spl["EXP_60"])
_, spl0,  spl0_q1,  spl0_q3  = aggregate(spl["EXP_0"])


# ====================== PLOT SETTINGS ======================
fontsize = 16
axis_fontsize = 18

# ====================== PLOT: SR ======================
plt.figure()
plt.plot(x60, sr60, marker="o", label="EXP_60 (Exploration)", color="tab:blue")
plt.fill_between(x60, sr60_q1, sr60_q3, alpha=0.25, color="tab:blue")

plt.plot(x0, sr0, marker="s", label="EXP_0 (Pure Exploitation)", color="tab:orange")
plt.fill_between(x0, sr0_q1, sr0_q3, alpha=0.25, color="tab:orange")

plt.xlabel("Semantic Retrieval Granularity (TOPK)", fontsize=fontsize)
plt.ylabel("Success Rate (SR)", fontsize=fontsize)
plt.title("SR vs Semantic Map Granularity (RQ3)", fontsize=fontsize)
plt.legend(fontsize=fontsize)
plt.grid(True)
plt.xticks(x60, fontsize=axis_fontsize)
plt.yticks(fontsize=axis_fontsize)
plt.tight_layout()


# ====================== PLOT: SPL ======================
plt.figure()
plt.plot(x60, spl60, marker="o", label="EXP_60 (Exploration)", color="tab:blue")
plt.fill_between(x60, spl60_q1, spl60_q3, alpha=0.25, color="tab:blue")

plt.plot(x0, spl0, marker="s", label="EXP_0 (Pure Exploitation)", color="tab:orange")
plt.fill_between(x0, spl0_q1, spl0_q3, alpha=0.25, color="tab:orange")

plt.xlabel("Semantic Retrieval Granularity (TOPK)", fontsize=fontsize)
plt.ylabel("Success weighted Path Length (SPL)", fontsize=fontsize)
plt.title("SPL vs Semantic Map Granularity (RQ3)", fontsize=fontsize)
plt.legend(fontsize=fontsize)
plt.grid(True)
plt.xticks(x60, fontsize=axis_fontsize)
plt.yticks(fontsize=axis_fontsize)
plt.tight_layout()

plt.show()
