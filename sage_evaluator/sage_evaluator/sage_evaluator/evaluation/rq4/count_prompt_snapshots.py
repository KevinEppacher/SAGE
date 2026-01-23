#!/usr/bin/env python3

from pathlib import Path
from collections import defaultdict
import re
import time

# =====================================================
# CONFIG
# =====================================================
SCENE_ROOT = Path(
    "/app/src/sage_evaluator/sage_datasets/matterport_isaac/"
    "00824-Dd4bFSTQ8gi/episodes/RQ4"
)

POS_TARGET = 3
NEG_TARGET = 7

# Folder format:
# YYYY-MM-DD_HHMM_POS_prompt
# YYYY-MM-DD_HHMM_NEG_prompt
SNAPSHOT_RE = re.compile(
    r"\d{4}-\d{2}-\d{2}_\d{4}_(POS|NEG)_([a-z0-9_]+)$"
)

# =====================================================
# HELPERS
# =====================================================
def scan_scene(scene_root: Path):
    """
    Scans filesystem and returns:
    counts[prompt]["POS"|"NEG"] = int
    """
    counts = defaultdict(lambda: {"POS": 0, "NEG": 0})

    if not scene_root.exists():
        return counts

    for d in scene_root.iterdir():
        if not d.is_dir():
            continue

        m = SNAPSHOT_RE.match(d.name)
        if not m:
            continue

        label, prompt = m.groups()
        counts[prompt][label] += 1

    return counts


def prompt_done(stats):
    return (
        stats["POS"] >= POS_TARGET
        and stats["NEG"] >= NEG_TARGET
    )


def print_status(counts):
    print("\n================ DATASET STATUS ================")
    for prompt, stats in sorted(counts.items()):
        print(
            f"{prompt:15s} | "
            f"POS {stats['POS']:2d}/{POS_TARGET} | "
            f"NEG {stats['NEG']:2d}/{NEG_TARGET}"
        )
    print("================================================")


def next_required_action(counts):
    """
    Returns:
        (prompt, "POS"|"NEG") or (None, None) if done
    """
    for prompt, stats in counts.items():
        if stats["POS"] < POS_TARGET:
            return prompt, "POS"
        if stats["NEG"] < NEG_TARGET:
            return prompt, "NEG"
    return None, None


# =====================================================
# MAIN LOOP
# =====================================================
def main():
    print("\n📊 Dataset collection monitor started")
    print(f"Scene: {SCENE_ROOT}")
    print(f"Target per prompt: POS={POS_TARGET}, NEG={NEG_TARGET}\n")

    while True:
        counts = scan_scene(SCENE_ROOT)

        if not counts:
            print("⚠️  No snapshots yet. Start annotating in the GUI.")
            time.sleep(1.0)
            continue

        print_status(counts)

        prompt, action = next_required_action(counts)

        if prompt is None:
            print("\n✅ DATASET COMPLETE FOR THIS SCENE")
            break

        if action == "POS":
            print(
                f"\n→ NEXT ACTION: "
                f"target_present for prompt '{prompt}'"
            )
        else:
            print(
                f"\n→ NEXT ACTION: "
                f"target_not_present for prompt '{prompt}'"
            )

        print("   (Annotate in GUI – monitoring continues automatically)")
        time.sleep(1.0)


# =====================================================
# ENTRY POINT
# =====================================================
if __name__ == "__main__":
    main()
