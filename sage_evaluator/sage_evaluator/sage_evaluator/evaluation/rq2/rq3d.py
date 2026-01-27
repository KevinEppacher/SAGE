import json
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

# ====================== CONFIG ======================
ROOT = Path("/app/src/sage_evaluator/sage_datasets/matterport_isaac")
RQ = "RQ2"
MAX_OBJECTS = 5
EXCLUDE_SCENES = {
    "example_scene",
    # "00800-TEEsavR23oF",
    # "00813-svBbv1Pavdk",
    # "00814-p53SfW6mjZe",
    # "00824-Dd4bFSTQ8gi",
    # "00848-ziup5kvtCCR",
    # "00876-mv2HUxq3B53",

}

# ====================================================

def parse_hyperparam(exp_dir: Path) -> float:
    # EXP_MEM_40_60 -> exploitation weight = 0.6
    _, _, explore, exploit = exp_dir.name.split("_")
    explore = float(explore)
    exploit = float(exploit)
    return exploit / (explore + exploit)

# exploitation_weight -> object_index -> list[SPL]
spl_by_h_and_index = {}

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

            with open(metrics_file, "r") as f:
                data = json.load(f)

            spl_by_h_and_index.setdefault(
                h, {i: [] for i in range(1, MAX_OBJECTS + 1)}
            )

            for idx, entry in enumerate(data.values(), start=1):
                if idx > MAX_OBJECTS:
                    break
                if "SPL" in entry:
                    spl_by_h_and_index[h][idx].append(entry["SPL"])

# ====================== SAFETY CHECK ======================
if not spl_by_h_and_index:
    raise RuntimeError("No SPL data found.")

# ====================== PLOTTING ======================
fontsize = 16
axis_fontsize = 18

x = np.arange(1, MAX_OBJECTS + 1)
cmap = plt.cm.viridis
norm = plt.Normalize(
    vmin=min(spl_by_h_and_index.keys()),
    vmax=max(spl_by_h_and_index.keys())
)

plt.figure()

for h in sorted(spl_by_h_and_index.keys()):
    means = []
    stds = []

    for idx in x:
        vals = np.array(spl_by_h_and_index[h][idx])
        if len(vals) == 0:
            means.append(np.nan)
            stds.append(0.0)
        else:
            means.append(np.mean(vals))
            stds.append(np.std(vals))

    color = cmap(norm(h))

    plt.plot(
        x,
        means,
        marker="o",
        color=color,
        label=f"{h:.2f}"
    )
    plt.fill_between(
        x,
        np.array(means) - np.array(stds),
        np.array(means) + np.array(stds),
        color=color,
        alpha=0.2
    )

plt.xlabel("Object index within episode", fontsize=fontsize)
plt.ylabel("Average SPL", fontsize=fontsize)
plt.title(
    "SPL progression over multi-object episodes\n"
    "Effect of exploitation (memory) weight",
    fontsize=fontsize
)

plt.xticks(x, fontsize=axis_fontsize)
plt.yticks(fontsize=axis_fontsize)
plt.grid(True)

cbar = plt.colorbar(
    plt.cm.ScalarMappable(norm=norm, cmap=cmap)
)
cbar.set_label("Exploitation weight", fontsize=fontsize)
cbar.ax.tick_params(labelsize=axis_fontsize)

plt.legend(
    title="Exploitation weight",
    fontsize=12,
    title_fontsize=12
)

plt.tight_layout()
plt.show()


import numpy as np
import matplotlib.pyplot as plt

# ====================== DELTA SPL COMPUTATION ======================
# exploitation_weight -> object_index -> list[ΔSPL]
delta_spl_by_h_and_index = {}

for h, index_dict in spl_by_h_and_index.items():
    delta_spl_by_h_and_index.setdefault(
        h, {i: [] for i in range(1, MAX_OBJECTS + 1)}
    )

    # SPL(1) values per episode are already mixed,
    # so we compute deltas per-sample
    base_vals = index_dict[1]

    for idx in range(1, MAX_OBJECTS + 1):
        vals = index_dict[idx]

        # Pairwise subtraction (truncate to shortest length)
        n = min(len(base_vals), len(vals))
        if n == 0:
            continue

        deltas = np.array(vals[:n]) - np.array(base_vals[:n])
        delta_spl_by_h_and_index[h][idx].extend(deltas)

# ====================== PLOTTING ======================
fontsize = 16
axis_fontsize = 18

x = np.arange(1, MAX_OBJECTS + 1)
cmap = plt.cm.viridis
norm = plt.Normalize(
    vmin=min(delta_spl_by_h_and_index.keys()),
    vmax=max(delta_spl_by_h_and_index.keys())
)

plt.figure()

for h in sorted(delta_spl_by_h_and_index.keys()):
    means = []
    stds = []

    for idx in x:
        vals = np.array(delta_spl_by_h_and_index[h][idx])
        if len(vals) == 0:
            means.append(np.nan)
            stds.append(0.0)
        else:
            means.append(np.mean(vals))
            stds.append(np.std(vals))

    color = cmap(norm(h))

    plt.plot(
        x,
        means,
        marker="o",
        color=color,
        label=f"{h:.2f}"
    )
    plt.fill_between(
        x,
        np.array(means) - np.array(stds),
        np.array(means) + np.array(stds),
        color=color,
        alpha=0.2
    )

plt.axhline(0.0, linestyle="--", linewidth=1.5)
plt.xlabel("Object index within episode", fontsize=fontsize)
plt.ylabel("ΔSPL relative to first object", fontsize=fontsize)
plt.title(
    "Relative SPL change over multi-object episodes\n"
    "Normalized to episode start",
    fontsize=fontsize
)

plt.xticks(x, fontsize=axis_fontsize)
plt.yticks(fontsize=axis_fontsize)
plt.grid(True)

cbar = plt.colorbar(
    plt.cm.ScalarMappable(norm=norm, cmap=cmap)
)
cbar.set_label("Exploitation weight", fontsize=fontsize)
cbar.ax.tick_params(labelsize=axis_fontsize)

plt.legend(
    title="Exploitation weight",
    fontsize=12,
    title_fontsize=12
)

plt.tight_layout()
plt.show()
