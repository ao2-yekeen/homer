#!/usr/bin/env python3
"""Build and query the conservative HOM-9 SO-ARM reachability lookup.

The map is a compact, versioned JSON file derived from a workspace point
cloud.  It is a feasibility pre-filter only: it never commands the arm and a
``reachable`` result is not authority to move hardware.
"""

from __future__ import annotations

import argparse
import json
import math
from collections.abc import Iterable, Sequence
from pathlib import Path
from typing import Any


SCHEMA_VERSION = 1
REACHABLE = "reachable"
MARGINAL = "marginal"
UNREACHABLE = "unreachable"
CLASSES = (REACHABLE, MARGINAL, UNREACHABLE)


def voxel_key(point: Sequence[float], voxel_size_m: float) -> tuple[int, int, int]:
    if voxel_size_m <= 0:
        raise ValueError("voxel size must be positive")
    return tuple(math.floor(value / voxel_size_m + 0.5) for value in point)  # type: ignore[return-value]


def neighbouring_voxels(key: tuple[int, int, int], radius: int) -> Iterable[tuple[int, int, int]]:
    for x in range(key[0] - radius, key[0] + radius + 1):
        for y in range(key[1] - radius, key[1] + radius + 1):
            for z in range(key[2] - radius, key[2] + radius + 1):
                yield x, y, z


def make_lookup(
    points: Iterable[Sequence[float]], voxel_size_m: float, marginal_margin_m: float
) -> dict[str, Any]:
    """Build a lookup with sampled cells reachable and adjacent cells marginal.

    A sampled cell is the only class called ``reachable``.  Cells inside the
    configurable margin around that conservative sampled set are ``marginal``
    and must be physically validated before planning.  Every other XYZ query
    is ``unreachable`` by default, including points outside the finite map.
    """
    if marginal_margin_m < 0:
        raise ValueError("marginal margin must be non-negative")
    reachable = {voxel_key(point, voxel_size_m) for point in points}
    if not reachable:
        raise ValueError("at least one workspace point is required")
    radius = math.ceil(marginal_margin_m / voxel_size_m)
    marginal = {
        candidate
        for key in reachable
        for candidate in neighbouring_voxels(key, radius)
        if candidate not in reachable
    }
    return {
        "schema_version": SCHEMA_VERSION,
        "frame_id": "base_footprint",
        "voxel_size_m": voxel_size_m,
        "marginal_margin_m": marginal_margin_m,
        "classification": {
            REACHABLE: "sampled, collision-filtered workspace voxel",
            MARGINAL: "within the configured voxel margin of sampled workspace; REQUIRES_HARDWARE_TEST",
            UNREACHABLE: "all cells not explicitly stored as reachable or marginal",
        },
        "limitations": [
            "This map is a reachability pre-filter, not a motion planner or safety approval.",
            "Unmodelled platform, cable, self-collision, and servo-to-URDF calibration constraints remain excluded.",
            "Marginal cells require physical validation before use in a motion decision.",
        ],
        "voxels": {
            REACHABLE: sorted(reachable),
            MARGINAL: sorted(marginal),
        },
    }


def classify(lookup: dict[str, Any], point: Sequence[float]) -> str:
    """Return ``reachable``, ``marginal``, or ``unreachable`` for XYZ metres."""
    if lookup.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("unsupported reachability-map schema")
    key = voxel_key(point, float(lookup["voxel_size_m"]))
    voxels = lookup["voxels"]
    if key in {tuple(value) for value in voxels[REACHABLE]}:
        return REACHABLE
    if key in {tuple(value) for value in voxels[MARGINAL]}:
        return MARGINAL
    return UNREACHABLE


def read_ascii_ply(path: Path) -> list[tuple[float, float, float]]:
    """Read x/y/z vertices from the ASCII PLY emitted by pybullet_sim.py."""
    with path.open(encoding="utf-8") as source:
        header = []
        for line in source:
            header.append(line.rstrip())
            if line.rstrip() == "end_header":
                break
        else:
            raise ValueError("PLY header has no end_header")
        if "format ascii 1.0" not in header:
            raise ValueError("only ASCII PLY is supported")
        count_line = next((line for line in header if line.startswith("element vertex ")), None)
        if count_line is None:
            raise ValueError("PLY header has no vertex count")
        count = int(count_line.split()[-1])
        points = []
        for _ in range(count):
            values = source.readline().split()
            if len(values) < 3:
                raise ValueError("PLY contains fewer vertices than declared")
            points.append((float(values[0]), float(values[1]), float(values[2])))
    return points


def write_lookup(path: Path, lookup: dict[str, Any], source: str) -> None:
    output = dict(lookup)
    output["source"] = source
    output["counts"] = {
        REACHABLE: len(output["voxels"][REACHABLE]),
        MARGINAL: len(output["voxels"][MARGINAL]),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def read_lookup(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def validate_samples(lookup: dict[str, Any], samples_path: Path) -> dict[str, Any]:
    """Compare append-only physical records with lookup predictions.

    A marginal prediction is intentionally reported as needing review rather
    than counted as a match for either physical result.
    """
    compared = matches = needs_review = 0
    rows = []
    for line_number, line in enumerate(samples_path.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        sample = json.loads(line)
        if sample.get("frame_id") != lookup.get("frame_id"):
            raise ValueError(f"sample line {line_number} has an incompatible frame")
        observed = sample.get("result")
        if observed not in (REACHABLE, UNREACHABLE):
            raise ValueError(f"sample line {line_number} has invalid result")
        predicted = classify(lookup, sample["gripper_center_xyz_m"])
        compared += 1
        review = predicted == MARGINAL
        matched = (predicted == observed) and not review
        matches += int(matched)
        needs_review += int(review)
        rows.append({"sample_id": sample.get("sample_id"), "observed": observed,
                     "predicted": predicted, "match": matched, "requires_review": review})
    return {"samples_compared": compared, "matches": matches,
            "requires_review": needs_review, "results": rows}


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(description=__doc__)
    commands = root.add_subparsers(dest="command", required=True)
    build = commands.add_parser("build", help="create a lookup from an ASCII PLY workspace cloud")
    build.add_argument("--points", type=Path, required=True, help="ASCII PLY from simulation/pybullet_sim.py --workspace")
    build.add_argument("--output", type=Path, default=Path("data/reachability/reachability_map.json"))
    build.add_argument("--voxel-size-m", type=float, default=0.01)
    build.add_argument("--marginal-margin-m", type=float, default=0.03)
    query = commands.add_parser("query", help="classify a gripper-centre XYZ point in base_footprint")
    query.add_argument("--map", type=Path, default=Path("data/reachability/reachability_map.json"))
    query.add_argument("--x-m", type=float, required=True)
    query.add_argument("--y-m", type=float, required=True)
    query.add_argument("--z-m", type=float, required=True)
    validate = commands.add_parser("validate", help="compare physical samples with lookup predictions")
    validate.add_argument("--map", type=Path, default=Path("data/reachability/reachability_map.json"))
    validate.add_argument("--samples", type=Path, default=Path("data/reachability/workspace_samples.jsonl"))
    return root


def main() -> int:
    args = parser().parse_args()
    if args.command == "build":
        lookup = make_lookup(read_ascii_ply(args.points), args.voxel_size_m, args.marginal_margin_m)
        write_lookup(args.output, lookup, str(args.points))
        print(json.dumps({"output": str(args.output), **read_lookup(args.output)["counts"]}, sort_keys=True))
    elif args.command == "query":
        lookup = read_lookup(args.map)
        result = classify(lookup, (args.x_m, args.y_m, args.z_m))
        print(json.dumps({"frame_id": lookup["frame_id"], "xyz_m": [args.x_m, args.y_m, args.z_m], "reachability": result}))
    else:
        print(json.dumps(validate_samples(read_lookup(args.map), args.samples), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
