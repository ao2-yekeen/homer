#!/usr/bin/env python3
"""Record a manually observed SO-ARM workspace sample without commanding motion.

The operator manually positions the supported arm using the existing guarded
teleoperation path, measures the gripper-centre point in `base_footprint`, and
records whether that point was reachable with all required clearances.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
from pathlib import Path
from typing import Sequence


VALID_RESULTS = {"reachable", "unreachable"}


def parse_joint_ticks(values: Sequence[str] | None) -> list[int] | None:
    """Return six integer servo positions, or None when they were not logged."""
    if values is None:
        return None
    if len(values) != 6:
        raise ValueError("joint ticks must contain exactly six values")
    ticks = [int(value) for value in values]
    if any(value < 0 or value > 4095 for value in ticks):
        raise ValueError("joint ticks must be within the STS 0..4095 range")
    return ticks


def make_sample(args: argparse.Namespace) -> dict[str, object]:
    if args.result not in VALID_RESULTS:
        raise ValueError(f"result must be one of: {', '.join(sorted(VALID_RESULTS))}")
    if not args.confirm_physical_observation:
        raise ValueError("--confirm-physical-observation is required")
    if not args.reason.strip():
        raise ValueError("--reason is required to preserve the observation evidence")
    return {
        "sample_id": args.sample_id,
        "timestamp_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "frame_id": "base_footprint",
        "gripper_center_xyz_m": [args.x_m, args.y_m, args.z_m],
        "result": args.result,
        "joint_ticks": parse_joint_ticks(args.joint_ticks),
        "evidence": args.reason,
        "clearance_checked": {
            "mast": args.mast_clear,
            "base": args.base_clear,
            "camera": args.camera_clear,
            "platform": args.platform_clear,
            "cables": args.cables_clear,
        },
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sample-id", required=True)
    parser.add_argument("--x-m", type=float, required=True)
    parser.add_argument("--y-m", type=float, required=True)
    parser.add_argument("--z-m", type=float, required=True)
    parser.add_argument("--result", required=True, choices=sorted(VALID_RESULTS))
    parser.add_argument("--reason", required=True)
    parser.add_argument("--joint-ticks", nargs=6, metavar=("J1", "J2", "J3", "J4", "J5", "J6"))
    parser.add_argument("--mast-clear", action="store_true")
    parser.add_argument("--base-clear", action="store_true")
    parser.add_argument("--camera-clear", action="store_true")
    parser.add_argument("--platform-clear", action="store_true")
    parser.add_argument("--cables-clear", action="store_true")
    parser.add_argument(
        "--confirm-physical-observation",
        action="store_true",
        help="attest that this sample was observed on the physical robot",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("data/reachability/workspace_samples.jsonl"),
        help="append-only JSONL observation file",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        sample = make_sample(args)
    except ValueError as exc:
        raise SystemExit(f"error: {exc}") from exc
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("a", encoding="utf-8") as output:
        output.write(json.dumps(sample, sort_keys=True) + "\n")
    print(f"Recorded {sample['sample_id']} in {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
