#!/usr/bin/env python3
"""Low-resource PyBullet simulation using the supplied robot v6 URDF.

The robot is moved kinematically from differential-drive commands. This keeps
the simulation light while preserving the supplied collision geometry and the
LiDAR frame. The emitted scan data can later be connected to a ROS adapter.
"""

from __future__ import annotations

import argparse
import math
import os
import time
from pathlib import Path

import pybullet as pb
import pybullet_data


URDF_NAME = "home_mobile_manipulator_v6_real_base_layout.urdf"
PROJECT_URDF = Path(__file__).with_name("urdf") / URDF_NAME
DOWNLOAD_URDF = Path("/mnt/c/Users/abdul/Downloads") / URDF_NAME
DEFAULT_URDF = PROJECT_URDF if PROJECT_URDF.exists() else DOWNLOAD_URDF
WHEEL_RADIUS_M = 0.0325
TRACK_WIDTH_M = 0.230
LIDAR_RAYS = 360
LIDAR_RANGE_M = 8.0


def add_box(center: tuple[float, float, float], size: tuple[float, float, float]) -> int:
    collision = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[v / 2 for v in size])
    visual = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[v / 2 for v in size])
    return pb.createMultiBody(0, collision, visual, center)


def make_test_world() -> None:
    """Small maze-like world with only primitive collision objects."""
    wall_height = 0.35
    walls = [
        ((2.5, 0.0, wall_height / 2), (0.10, 5.0, wall_height)),
        ((-2.5, 0.0, wall_height / 2), (0.10, 5.0, wall_height)),
        ((0.0, 2.5, wall_height / 2), (5.0, 0.10, wall_height)),
        ((0.0, -2.5, wall_height / 2), (5.0, 0.10, wall_height)),
        ((0.7, 0.7, wall_height / 2), (1.2, 0.12, wall_height)),
        ((-0.8, -0.7, wall_height / 2), (0.12, 1.4, wall_height)),
    ]
    for center, size in walls:
        add_box(center, size)


def find_link(robot: int, name: str) -> int:
    for index in range(pb.getNumJoints(robot)):
        joint = pb.getJointInfo(robot, index)
        if joint[12].decode() == name:
            return index
    raise RuntimeError(f"URDF link not found: {name}")


def scan(robot: int, lidar_link: int, pose: tuple[float, float, float]) -> list[float]:
    state = pb.getLinkState(robot, lidar_link, computeForwardKinematics=True)
    origin = state[4]
    yaw = pose[2]
    rays_from = []
    rays_to = []
    for i in range(LIDAR_RAYS):
        angle = yaw + (2.0 * math.pi * i / LIDAR_RAYS)
        # Start outside the physical LiDAR housing. The supplied URDF has
        # collision geometry for the housing, so a ray starting at the joint
        # origin would otherwise report an immediate self-hit.
        direction = (math.cos(angle), math.sin(angle), 0.0)
        rays_from.append(tuple(origin[j] + 0.08 * direction[j] for j in range(3)))
        rays_to.append(
            (origin[0] + LIDAR_RANGE_M * math.cos(angle),
             origin[1] + LIDAR_RANGE_M * math.sin(angle), origin[2])
        )
    hits = pb.rayTestBatch(rays_from, rays_to, numThreads=0)
    return [
        LIDAR_RANGE_M if hit[0] < 0 or hit[0] == robot else hit[2] * LIDAR_RANGE_M
        for hit in hits
    ]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=Path(os.environ.get("ROBOT_URDF", DEFAULT_URDF)))
    parser.add_argument("--gui", action="store_true", help="Open the PyBullet window")
    parser.add_argument("--seconds", type=float, default=20.0)
    args = parser.parse_args()
    if not args.urdf.exists():
        raise SystemExit(f"URDF not found: {args.urdf}")

    client = pb.connect(pb.GUI if args.gui else pb.DIRECT)
    try:
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.setGravity(0, 0, -9.81)
        pb.loadURDF("plane.urdf")
        make_test_world()
        robot = pb.loadURDF(str(args.urdf), basePosition=(0, 0, 0.02), useFixedBase=False,
                            flags=pb.URDF_USE_INERTIA_FROM_FILE)
        lidar_link = find_link(robot, "rplidar_a1")

        x, y, theta = 0.0, 0.0, 0.0
        started = time.monotonic()
        last = started
        sample = 0
        while time.monotonic() - started < args.seconds:
            now = time.monotonic()
            dt = min(now - last, 0.05)
            last = now
            phase = (now - started) % 16.0
            linear, angular = (0.10, 0.0) if phase < 8.0 else (0.0, math.pi / 8.0)
            x += linear * math.cos(theta) * dt
            y += linear * math.sin(theta) * dt
            theta += angular * dt
            pb.resetBasePositionAndOrientation(robot, (x, y, 0.02), pb.getQuaternionFromEuler((0, 0, theta)))
            ranges = scan(robot, lidar_link, (x, y, theta))
            if sample % 20 == 0:
                print(f"t={now-started:5.1f}s pose=({x:+.3f}, {y:+.3f}, {theta:+.3f}) "
                      f"scan_min={min(ranges):.3f}m")
            sample += 1
            pb.stepSimulation()
            if args.gui:
                time.sleep(1.0 / 60.0)
    finally:
        pb.disconnect(client)


if __name__ == "__main__":
    main()

