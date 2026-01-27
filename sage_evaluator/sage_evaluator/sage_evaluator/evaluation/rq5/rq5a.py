import matplotlib.pyplot as plt
import numpy as np

# ======================
# DATA (MB)
# ======================
components = ["OpenFusion", "BLIP-2", "YOLO-E"]
vram_mb = np.array([6122, 2280, 1200])
total_vram = vram_mb.sum()

# ======================
# PLOT SETTINGS
# ======================
fontsize = 16
axis_fontsize = 18

# Inferno-like colors (dark → bright)
colors = ["#440154", "#31688E", "#35B779"]
# viridis-like colors (dark → bright)
# colors = ["#440154", "#21918C", "#FDE725"]

# ======================
# PLOT
# ======================
plt.figure(figsize=(9, 3))

left = 0
for comp, val, color in zip(components, vram_mb, colors):
    plt.barh(
        0,
        val,
        left=left,
        color=color,
        label=f"{comp} ({val} MB)"
    )

    # Centered annotation
    plt.text(
        left + val / 2,
        0,
        f"{comp}",
        ha="center",
        va="center",
        fontsize=fontsize - 2,
        color="white",
        fontweight="bold"
    )

    left += val

# ======================
# AXES & DECORATION
# ======================
plt.xlabel("GPU Memory Usage [MB]", fontsize=fontsize)
plt.ylabel("Total GPU\n Memory Usage", fontsize=fontsize, rotation=90)
plt.xticks(fontsize=axis_fontsize)
plt.yticks([])
plt.xlim(0, total_vram * 1.1)

plt.title(
    f"Stacked GPU Memory Footprint of System Components "
    f"(Total: {total_vram} MB)",
    fontsize=fontsize
)

plt.legend(
    loc="upper center",
    bbox_to_anchor=(0.5, -0.35),
    ncol=3,
    fontsize=fontsize - 2,
    frameon=False
)

# plt.grid(axis="x", linestyle="--", alpha=0.4)
plt.tight_layout()
plt.show()
