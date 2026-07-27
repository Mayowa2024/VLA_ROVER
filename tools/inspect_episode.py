#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path
import statistics


def print_pair_summary(label, values) -> None:
    first = [float(value[0]) for value in values]
    second = [float(value[1]) for value in values]
    nonzero = sum(
        abs(value[0]) > 1e-6 or abs(value[1]) > 1e-6
        for value in values
    )
    print(
        f"{label} first value:  "
        f"min={min(first):.4f}, max={max(first):.4f}, "
        f"mean={statistics.fmean(first):.4f}"
    )
    print(
        f"{label} second value: "
        f"min={min(second):.4f}, max={max(second):.4f}, "
        f"mean={statistics.fmean(second):.4f}"
    )
    print(f"{label} nonzero frames: {nonzero}/{len(values)}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("episode_dir", type=Path)
    args = parser.parse_args()

    episode_dir = args.episode_dir.expanduser()
    metadata_path = episode_dir / "metadata.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    rows = []
    with (episode_dir / "frames.jsonl").open(encoding="utf-8") as file:
        for line in file:
            if line.strip():
                rows.append(json.loads(line))

    images = list((episode_dir / "images").glob("*.jpg"))
    print(f"Episode: {episode_dir}")
    print(f"Rows: {len(rows)}")
    print(f"Images: {len(images)}")
    print(f"Finalized: {metadata.get('finalized', False)}")
    print(f"Configured rate: {metadata.get('sample_rate_hz', 'unknown')} Hz")

    if not rows:
        raise SystemExit("No frame records found.")

    frame_indices = [int(row["frame_index"]) for row in rows]
    if frame_indices != list(range(len(rows))):
        raise SystemExit("Frame indices are not contiguous.")

    numeric_values = [
        float(value)
        for row in rows
        for value in (
            row["timestamp_sec"],
            *row["action"],
            *row["state"],
        )
    ]
    if not all(math.isfinite(value) for value in numeric_values):
        raise SystemExit("A timestamp, action or state value is not finite.")

    instructions = sorted({row["instruction"] for row in rows})
    print(f"Instructions: {instructions}")
    print(f"State source: {rows[0]['state_source']}")
    start = float(rows[0]["timestamp_sec"])
    end = float(rows[-1]["timestamp_sec"])
    duration = end - start
    print(f"Duration: {duration:.3f} s")
    if len(rows) > 1 and duration > 0.0:
        print(f"Observed rate: {(len(rows) - 1) / duration:.3f} Hz")

    print_pair_summary("Action [linear.x, angular.z]", [
        row["action"] for row in rows
    ])
    print_pair_summary("State  [linear.x, angular.z]", [
        row["state"] for row in rows
    ])

    missing = [
        row["image"] for row in rows
        if not (episode_dir / row["image"]).exists()
    ]
    if missing:
        print(f"Missing images: {len(missing)}")
        raise SystemExit(1)

    if len(images) != len(rows):
        print("Warning: image count and row count differ.")
    else:
        print("Basic integrity check passed.")


if __name__ == "__main__":
    main()
