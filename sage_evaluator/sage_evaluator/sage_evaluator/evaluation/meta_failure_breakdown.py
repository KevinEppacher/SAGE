import json
from pathlib import Path
from collections import Counter, defaultdict

# ====================== CONFIG ======================
ROOT = Path("/app/src/sage_evaluator/sage_datasets/matterport_isaac")
RQ = "RQ2"
TARGET_EXP = "EXP_MEM_60_40"   # <-- change to EXP_MEM_40_60 when needed
EXCLUDE_SCENES = {"example_scene"}
# ====================================================

CALLED_STOP_REASONS = {"wrong_object", "seen_but_far", "moved_away"}
TIMEOUT_REASONS = {"not_seen", "ignored", "no_motion", "nav_failure"}

# ====================== STORAGE ======================
global_called = Counter()
global_timeout = Counter()
scene_called = defaultdict(Counter)
scene_timeout = defaultdict(Counter)

total_trials = 0
scene_trials = Counter()
episode_counter = Counter()

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

            episode_counter[scene.name] += 1

            with open(metrics_file, "r") as f:
                metrics = json.load(f)

            for entry in metrics.values():
                total_trials += 1
                scene_trials[scene.name] += 1

                reason = entry.get("failure_reason")
                if reason is None:
                    continue

                if reason == "nav_failure":
                    reason = "no_motion"

                if reason in CALLED_STOP_REASONS:
                    global_called[reason] += 1
                    scene_called[scene.name][reason] += 1
                elif reason in TIMEOUT_REASONS:
                    global_timeout[reason] += 1
                    scene_timeout[scene.name][reason] += 1

# ====================== GLOBAL SUMMARY ======================
called_total = sum(global_called.values())
timeout_total = sum(global_timeout.values())
failure_total = called_total + timeout_total

print("\n===================================================")
print(f"Failure summary for experiment: {TARGET_EXP}")
print("===================================================")
print(f"Total scenes: {len(scene_trials)}")
print(f"Total episodes: {sum(episode_counter.values())}")
print(f"Total trials: {total_trials}")
print(f"Total failures: {failure_total}")
print(f"  Called STOP: {called_total}")
print(f"  Timeout: {timeout_total}")

print("\n-- Global failure breakdown --")
for k, v in global_called.items():
    print(f"Called STOP | {k:15s}: {v}")
for k, v in global_timeout.items():
    print(f"Timeout     | {k:15s}: {v}")

# ====================== PER-SCENE TABLE ======================
print("\n===================================================")
print("Per-scene failure statistics (table-ready)")
print("===================================================")

header = (
    "Scene & Episodes & Trials & WrongObj & SeenFar & MovedAway & "
    "NotSeen & Ignored & NoMotion \\\\"
)
print(header)
print("\\hline")

for scene in sorted(scene_trials.keys()):
    c = scene_called[scene]
    t = scene_timeout[scene]

    row = (
        f"{scene} & "
        f"{episode_counter[scene]} & "
        f"{scene_trials[scene]} & "
        f"{c.get('wrong_object', 0)} & "
        f"{c.get('seen_but_far', 0)} & "
        f"{c.get('moved_away', 0)} & "
        f"{t.get('not_seen', 0)} & "
        f"{t.get('ignored', 0)} & "
        f"{t.get('no_motion', 0)} \\\\"
    )
    print(row)
