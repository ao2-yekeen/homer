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
WORLD_WALLS = [
    ((2.5, 0.0), (0.10, 5.0)),
    ((-2.5, 0.0), (0.10, 5.0)),
    ((0.0, 2.5), (5.0, 0.10)),
    ((0.0, -2.5), (5.0, 0.10)),
    ((0.7, 0.7), (1.2, 0.12)),
    ((-0.8, -0.7), (0.12, 1.4)),
]


def add_box(center: tuple[float, float, float], size: tuple[float, float, float]) -> int:
    collision = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[v / 2 for v in size])
    visual = pb.createVisualShape(pb.GEOM_BOX, halfExtents=[v / 2 for v in size])
    return pb.createMultiBody(0, collision, visual, center)


def make_test_world() -> None:
    """Small maze-like world with only primitive collision objects."""
    wall_height = 0.35
    for (x, y), (width, length) in WORLD_WALLS:
        center = (x, y, wall_height / 2)
        size = (width, length, wall_height)
        add_box(center, size)


def make_2d_view():
    """Create an optional low-resource top-down view."""
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    figure, axis = plt.subplots(figsize=(7, 7))
    axis.set_title("v6 robot — simulated RPLIDAR scan")
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.set_aspect("equal")
    axis.set_xlim(-3.0, 3.0)
    axis.set_ylim(-3.0, 3.0)
    for (x, y), (width, length) in WORLD_WALLS:
        axis.add_patch(Rectangle((x - width / 2, y - length / 2), width, length,
                                 facecolor="dimgray", edgecolor="black"))
    path, = axis.plot([], [], color="tab:blue", linewidth=1, label="path")
    points = axis.scatter([], [], s=3, color="tab:orange", label="lidar")
    robot_marker, = axis.plot([], [], "o", color="tab:green", markersize=8, label="robot")
    heading, = axis.plot([], [], color="tab:green", linewidth=2)
    axis.legend(loc="upper right")
    axis.grid(alpha=0.25)
    figure.tight_layout()
    plt.ion()
    if "agg" not in str(plt.get_backend()).lower():
        figure.show()
    return figure, axis, path, points, robot_marker, heading


def update_2d_view(view, pose, ranges) -> None:
    figure, axis, path, points, robot_marker, heading = view
    x, y, theta = pose
    angles = [theta + 2.0 * math.pi * i / LIDAR_RAYS for i in range(LIDAR_RAYS)]
    lidar_points = [(x + distance * math.cos(angle), y + distance * math.sin(angle))
                    for angle, distance in zip(angles, ranges) if distance < LIDAR_RANGE_M]
    if not hasattr(update_2d_view, "trail"):
        update_2d_view.trail = []
    update_2d_view.trail.append((x, y))
    path.set_data(*zip(*update_2d_view.trail))
    points.set_offsets(lidar_points or [[x, y]])
    robot_marker.set_data([x], [y])
    heading.set_data([x, x + 0.18 * math.cos(theta)], [y, y + 0.18 * math.sin(theta)])
    figure.canvas.draw_idle()
    figure.canvas.flush_events()


def make_pybullet_2d_view() -> dict:
    """Configure the PyBullet window as a live top-down 2D-like view."""
    pb.resetDebugVisualizerCamera(
        cameraDistance=7.2,
        cameraYaw=0.0,
        cameraPitch=-89.9,
        cameraTargetPosition=(0.0, 0.0, 0.0),
    )
    return {"scan_lines": [], "path_lines": [], "last_pose": None}


def update_pybullet_2d_view(view: dict, pose, ranges) -> None:
    """Draw a sparse live scan and path in the PyBullet top-down window."""
    x, y, theta = pose
    for line_id in view["scan_lines"] + view["path_lines"]:
        pb.removeUserDebugItem(line_id)
    view["scan_lines"] = []
    view["path_lines"] = []

    origin = (x, y, 0.05)
    # 90 lines are enough for a responsive low-resource display.
    for i in range(0, LIDAR_RAYS, 4):
        angle = theta + 2.0 * math.pi * i / LIDAR_RAYS
        distance = ranges[i]
        end = (x + distance * math.cos(angle), y + distance * math.sin(angle), 0.05)
        view["scan_lines"].append(
            pb.addUserDebugLine(origin, end, lineColorRGB=[1, 0.35, 0], lineWidth=1.0)
        )
    previous = view["last_pose"]
    if previous is not None:
        view["path_lines"].append(
            pb.addUserDebugLine((previous[0], previous[1], 0.06), origin,
                                lineColorRGB=[0, 0.4, 1], lineWidth=2.0)
        )
    view["last_pose"] = pose
    pb.addUserDebugLine(
        origin,
        (x + 0.2 * math.cos(theta), y + 0.2 * math.sin(theta), 0.05),
        lineColorRGB=[0, 1, 0], lineWidth=3.0, lifeTime=0.2,
    )


def make_live_2d_view() -> dict:
    """Create a real 2D window without rendering the PyBullet robot."""
    import pygame

    pygame.init()
    screen = pygame.display.set_mode((720, 720))
    pygame.display.set_caption("v6 robot - realtime 2D LiDAR")
    return {"pygame": pygame, "screen": screen, "clock": pygame.time.Clock(), "trail": []}


def update_live_2d_view(view: dict, pose, ranges) -> bool:
    """Render walls, path, robot heading, and LiDAR points in flat 2D."""
    pygame = view["pygame"]
    screen = view["screen"]
    scale, margin = 110.0, 30

    def pixel(point):
        return (int(margin + (point[0] + 3.0) * scale),
                int(690 - (point[1] + 3.0) * scale))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return False
    x, y, theta = pose
    view["trail"].append((x, y))
    screen.fill((248, 248, 248))
    for (wall_x, wall_y), (width, length) in WORLD_WALLS:
        left_top = pixel((wall_x - width / 2, wall_y + length / 2))
        pygame.draw.rect(screen, (80, 80, 80),
                         (left_top[0], left_top[1], int(width * scale), int(length * scale)))
    if len(view["trail"]) > 1:
        pygame.draw.lines(screen, (40, 110, 210), False,
                          [pixel(point) for point in view["trail"]], 2)
    for i in range(0, LIDAR_RAYS, 2):
        angle = theta + 2.0 * math.pi * i / LIDAR_RAYS
        distance = ranges[i]
        if distance < LIDAR_RANGE_M:
            pygame.draw.circle(screen, (255, 125, 20),
                               pixel((x + distance * math.cos(angle),
                                      y + distance * math.sin(angle))), 2)
    center = pixel((x, y))
    pygame.draw.circle(screen, (35, 165, 55), center, 9)
    pygame.draw.line(screen, (20, 120, 40), center,
                     pixel((x + 0.22 * math.cos(theta), y + 0.22 * math.sin(theta))), 3)
    pygame.display.flip()
    view["clock"].tick(30)
    return True


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
    parser.add_argument("--2d-gui", dest="view_2d_gui", action="store_true",
                        help="Open a realtime PyBullet top-down 2D-like view")
    parser.add_argument("--2d-live", dest="view_2d_live", action="store_true",
                        help="Open a genuine realtime flat 2D window")
    parser.add_argument("--2d", dest="view_2d", action="store_true",
                        help="Open a lightweight top-down scan view")
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--save-2d", type=Path, metavar="PNG",
                        help="Save a final 2D view image (useful on headless systems)")
    args = parser.parse_args()
    if not args.urdf.exists():
        raise SystemExit(f"URDF not found: {args.urdf}")

    view = make_2d_view() if args.view_2d else None
    use_2d_gui = args.view_2d_gui
    live_2d_view = make_live_2d_view() if args.view_2d_live else None
    client = pb.connect(pb.GUI if args.gui or use_2d_gui else pb.DIRECT)
    try:
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.setGravity(0, 0, -9.81)
        pb.loadURDF("plane.urdf")
        make_test_world()
        robot = pb.loadURDF(str(args.urdf), basePosition=(0, 0, 0.02), useFixedBase=False,
                            flags=pb.URDF_USE_INERTIA_FROM_FILE)
        pybullet_2d_view = make_pybullet_2d_view() if use_2d_gui else None
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
            if live_2d_view and not update_live_2d_view(live_2d_view, (x, y, theta), ranges):
                break
            if pybullet_2d_view:
                update_pybullet_2d_view(pybullet_2d_view, (x, y, theta), ranges)
            if view and sample % 3 == 0:
                update_2d_view(view, (x, y, theta), ranges)
            if sample % 20 == 0:
                print(f"t={now-started:5.1f}s pose=({x:+.3f}, {y:+.3f}, {theta:+.3f}) "
                      f"scan_min={min(ranges):.3f}m")
            sample += 1
            pb.stepSimulation()
            if args.gui:
                time.sleep(1.0 / 60.0)
            elif view:
                time.sleep(1.0 / 30.0)
        if view and args.save_2d:
            view[0].savefig(args.save_2d, dpi=150)
            print(f"saved 2D view to {args.save_2d}")
    finally:
        pb.disconnect(client)


if __name__ == "__main__":
    main()
