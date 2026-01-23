import json
from pathlib import Path
from collections import Counter, defaultdict
import plotly.graph_objects as go
import re

# ====================== CONFIG ======================
ROOT = Path("/app/src/sage_evaluator/sage_datasets/matterport_isaac")
RQ = "RQ3"
EXCLUDE_SCENES = {"example_scene"}
TOPK_PATTERN = re.compile(r"TOPK_(\d+)_EXP_(\d+)")
# ====================================================

# -------- Failure taxonomy --------
CALLED_STOP_REASONS = {"wrong_object", "seen_but_far", "moved_away"}
TIMEOUT_REASONS = {"not_seen", "ignored", "no_motion", "nav_failure"}
# ---------------------------------

# mode -> counters
called_stop = defaultdict(Counter)
timeout = defaultdict(Counter)
total_trials = Counter()

# ====================== DATA COLLECTION ======================
for scene in ROOT.iterdir():
    if not scene.is_dir() or scene.name in EXCLUDE_SCENES:
        continue

    rq_root = scene / "episodes" / RQ
    if not rq_root.exists():
        continue

    for episode_dir in rq_root.iterdir():
        if not episode_dir.is_dir():
            continue

        for exp in episode_dir.iterdir():
            m = TOPK_PATTERN.match(exp.name)
            if not m:
                continue

            exp_weight = int(m.group(2))
            mode = f"EXP_{exp_weight}"

            metrics_file = exp / "metrics.json"
            if not metrics_file.exists():
                continue

            with open(metrics_file, "r") as f:
                metrics = json.load(f)

            for entry in metrics.values():
                total_trials[mode] += 1
                reason = entry.get("failure_reason")
                if not reason:
                    continue

                if reason == "nav_failure":
                    reason = "no_motion"

                if reason in CALLED_STOP_REASONS:
                    called_stop[mode][reason] += 1
                elif reason in TIMEOUT_REASONS:
                    timeout[mode][reason] += 1

# ====================== SANKEY FUNCTION ======================
def plot_sankey(mode):
    total = total_trials[mode]
    if total == 0:
        return

    cs_total = sum(called_stop[mode].values())
    to_total = sum(timeout[mode].values())
    fail_total = cs_total + to_total

    def pct(x):
        return 100.0 * x / total

    labels = [
        f"Failures\n{pct(fail_total):.1f}%",
        f"Called STOP\n{pct(cs_total):.1f}%",
        f"Timeout\n{pct(to_total):.1f}%",
        f"Stopped at\nwrong object\n{pct(called_stop[mode]['wrong_object']):.1f}%",
        f"Saw goal\n{pct(called_stop[mode]['seen_but_far'] + called_stop[mode]['moved_away']):.1f}%",
        f"Stopped too far\nfrom goal\n{pct(called_stop[mode]['seen_but_far']):.1f}%",
        f"Moved away\nfrom goal\n{pct(called_stop[mode]['moved_away']):.1f}%",
        f"Didn't see\ngoal\n{pct(timeout[mode]['not_seen']):.1f}%",
        f"Ignored\ngoal\n{pct(timeout[mode]['ignored']):.1f}%",
        f"Navigation failure\n{pct(timeout[mode]['no_motion']):.1f}%",
    ]

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
        cs_total,
        to_total,
        called_stop[mode]['wrong_object'],
        called_stop[mode]['seen_but_far'] + called_stop[mode]['moved_away'],
        called_stop[mode]['seen_but_far'],
        called_stop[mode]['moved_away'],
        timeout[mode]['not_seen'],
        timeout[mode]['ignored'],
        timeout[mode]['no_motion'],
    ]

    # ---------- NODE COLORS (same as before) ----------
    node_colors = [
        "#2EC4C9",  # Failures (turquoise)
        "#1F77B4",  # Called STOP (blue)
        "#FF7F0E",  # Timeout (orange)
        "#FFD700",  # Wrong object (yellow)
        "#D62728",  # Saw goal (red)
        "#2CA02C",  # Too far (green)
        "#9467BD",  # Moved away (purple)
        "#8C564B",  # Not seen (brown)
        "#E377C2",  # Ignored (pink)
        "#4C566A",  # Navigation failure (gray)
    ]

    # ---------- LINK COLORS (same logic as before) ----------
    link_colors = [
        "rgba(46,196,201,0.35)",  # Failures -> Called STOP
        "rgba(46,196,201,0.35)",  # Failures -> Timeout

        "rgba(23,78,135,0.7)",   # Called STOP -> Wrong object
        "rgba(23,78,135,0.7)",   # Called STOP -> Saw goal

        "rgba(44,160,44,0.35)",  # Saw goal -> Too far
        "rgba(148,103,189,0.35)",# Saw goal -> Moved away

        "rgba(255,127,14,0.35)", # Timeout -> Not seen
        "rgba(255,127,14,0.35)", # Timeout -> Ignored
        "rgba(255,127,14,0.35)", # Timeout -> No motion
    ]

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

    if mode == "EXP_60":
        mode = "60% Exploration"
    elif mode == "EXP_0":
        mode = "100% Exploitation"
    else:
        mode = mode

    fig.update_layout(
        title=dict(
            text=f"Failure Mode Breakdown – {mode}",
            x=0.5,
            xanchor="center",
            font=dict(size=22)
        ),
        font=dict(size=22),
        margin=dict(l=20, r=20, t=80, b=20)
    )

    fig.show()


# ====================== RUN ======================
plot_sankey("EXP_60")
plot_sankey("EXP_0")
