import json
from pathlib import Path
from collections import Counter
import plotly.graph_objects as go

# ====================== CONFIG ======================
ROOT = Path("/app/src/sage_evaluator/sage_datasets/matterport_isaac")
RQ = "RQ2"
EXCLUDE_SCENES = {"example_scene"}

TARGET_EXP = "EXP_MEM_40_60"   # <-- CHANGE TO "EXP_MEM_40_60"
TITLE = "40% Exploration"   # <-- CHANGE TO "40% Exploration"

# ====================================================

# -------- Failure taxonomy (SOURCE OF TRUTH) --------
CALLED_STOP_REASONS = {
    "wrong_object",
    "seen_but_far",
    "moved_away",
}

TIMEOUT_REASONS = {
    "not_seen",
    "ignored",
    "no_motion",
    "nav_failure",   # legacy label
}
# ---------------------------------------------------

called_stop_counter = Counter()
timeout_counter = Counter()
total_trials = 0

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
            if exp.name != TARGET_EXP:
                continue

            metrics_file = exp / "metrics.json"
            if not metrics_file.exists():
                continue

            with open(metrics_file, "r") as f:
                metrics = json.load(f)

            for entry in metrics.values():
                total_trials += 1

                reason = entry.get("failure_reason")
                if reason is None:
                    continue

                if reason == "nav_failure":
                    reason = "no_motion"

                if reason in CALLED_STOP_REASONS:
                    called_stop_counter[reason] += 1
                elif reason in TIMEOUT_REASONS:
                    timeout_counter[reason] += 1

# ====================== SAFETY CHECK ======================
if total_trials == 0:
    raise RuntimeError(f"No trials found for {TARGET_EXP}")

# ====================== AGGREGATION ======================
called_stop_total = sum(called_stop_counter.values())
timeout_total = sum(timeout_counter.values())
failure_total = called_stop_total + timeout_total

def pct(x):
    return 100.0 * x / total_trials

print(f"\n=== Failure statistics for {TARGET_EXP} ===")
print(f"Total trials: {total_trials}")
print(f"Failures: {failure_total} ({pct(failure_total):.1f}%)")

# ====================== SANKEY LABELS ======================
labels = [
    f"Failures\n{pct(failure_total):.1f}%",
    f"Called STOP\n{pct(called_stop_total):.1f}%",
    f"Timeout\n{pct(timeout_total):.1f}%",

    f"Stopped at\nwrong object\n{pct(called_stop_counter.get('wrong_object', 0)):.1f}%",
    f"Saw goal\n{pct(called_stop_counter.get('seen_but_far', 0) + called_stop_counter.get('moved_away', 0)):.1f}%",

    f"Stopped too far\nfrom goal object\n{pct(called_stop_counter.get('seen_but_far', 0)):.1f}%",
    f"Moved away\nfrom goal object\n{pct(called_stop_counter.get('moved_away', 0)):.1f}%",

    f"Didn't see\ngoal object\n{pct(timeout_counter.get('not_seen', 0)):.1f}%",
    f"Ignored\ngoal object\n{pct(timeout_counter.get('ignored', 0)):.1f}%",
    f"Navigation failure\n(no motion)\n{pct(timeout_counter.get('no_motion', 0)):.1f}%",
]

# ====================== SANKEY LINKS ======================
sources = [
    0, 0,
    1, 1,
    4, 4,
    2, 2, 2
]

targets = [
    1, 2,
    3, 4,
    5, 6,
    7, 8, 9
]

values = [
    called_stop_total,
    timeout_total,

    called_stop_counter.get("wrong_object", 0),
    called_stop_counter.get("seen_but_far", 0) + called_stop_counter.get("moved_away", 0),

    called_stop_counter.get("seen_but_far", 0),
    called_stop_counter.get("moved_away", 0),

    timeout_counter.get("not_seen", 0),
    timeout_counter.get("ignored", 0),
    timeout_counter.get("no_motion", 0),
]

# ====================== COLORS (CONSISTENT) ======================
node_colors = [
    "#2EC4C9",
    "#1F77B4",
    "#FF7F0E",
    "#FFD700",
    "#D62728",
    "#2CA02C",
    "#9467BD",
    "#8C564B",
    "#E377C2",
    "#4C566A",
]

link_colors = [
    "rgba(46,196,201,0.35)",
    "rgba(46,196,201,0.35)",
    "rgba(23,78,135,0.7)",
    "rgba(23,78,135,0.7)",
    "rgba(44,160,44,0.35)",
    "rgba(148,103,189,0.35)",
    "rgba(255,127,14,0.35)",
    "rgba(255,127,14,0.35)",
    "rgba(255,127,14,0.35)",
]

# ====================== PLOT ======================
fig = go.Figure(go.Sankey(
    arrangement="freeform",
    node=dict(
        label=labels,
        color=node_colors,
        pad=20,
        thickness=18,
        line=dict(color="black", width=0.3)
    ),
    link=dict(
        source=sources,
        target=targets,
        value=values,
        color=link_colors
    )
))

fig.update_layout(
    title=dict(
        text=f"Failure Mode Breakdown: {TITLE} (RQ2)",
        x=0.5,
        xanchor="center",
        font=dict(size=22)
    ),
    font=dict(size=22),
    margin=dict(l=20, r=20, t=80, b=20)
)

fig.show()
