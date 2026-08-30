#!/usr/bin/env python3
"""Low-resource PyBullet simulation using the supplied robot v6 URDF.

The robot is moved kinematically from differential-drive commands. This keeps
the simulation light while preserving the supplied collision geometry and the
LiDAR frame. The emitted scan data can later be connected to a ROS adapter.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
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
LIDAR_NOISE_STD_M = 0.01
WHEEL_NOISE_FRACTION = 0.02
WHEEL_SCALE_ERROR = 0.01
WORKSPACE_JOINT_NAMES = (
    "shoulder_yaw_joint",
    "shoulder_pitch_joint",
    "elbow_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
    "gripper_joint",
)
WORKSPACE_OBSTACLE_LINK_NAMES = (
    "lower_waffle_plate",
    "upper_waffle_plate",
    "mast_link",
    "arm_base_mount",
    "mg996_neck_servo",
)
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


def joint_indices(robot: int, names: tuple[str, ...] = WORKSPACE_JOINT_NAMES) -> list[int]:
    """Find the calibrated URDF joints that determine the gripper position."""
    indices = {}
    for index in range(pb.getNumJoints(robot)):
        info = pb.getJointInfo(robot, index)
        indices[info[1].decode()] = index
    missing = [name for name in names if name not in indices]
    if missing:
        raise RuntimeError(f"URDF workspace joints not found: {', '.join(missing)}")
    return [indices[name] for name in names]


def halton(index: int, base: int) -> float:
    """Return a deterministic low-discrepancy value in [0, 1)."""
    result, fraction = 0.0, 1.0
    while index:
        fraction /= base
        result += fraction * (index % base)
        index //= base
    return result


def gripper_centre(robot: int, gripper_link: int) -> tuple[float, float, float]:
    """Return the centre of the gripper visual volume in base_footprint.

    The v6 URDF places the gripper box centre at x=0.050 m in its link frame.
    Keeping that offset explicit means the workspace cloud describes the useful
    gripper volume, rather than only the gripper-joint origin.
    """
    state = pb.getLinkState(robot, gripper_link, computeForwardKinematics=True)
    return tuple(pb.multiplyTransforms(state[4], state[5], (0.050, 0.0, 0.0), (0, 0, 0, 1))[0])


def link_indices_by_name(robot: int, names: tuple[str, ...]) -> set[int]:
    """Return link indices for named URDF links, rejecting incomplete models."""
    indices = {pb.getJointInfo(robot, index)[12].decode(): index for index in range(pb.getNumJoints(robot))}
    missing = [name for name in names if name not in indices]
    if missing:
        raise RuntimeError(f"URDF workspace obstacle links not found: {', '.join(missing)}")
    return {indices[name] for name in names}


def collides_with_workspace_obstacle(robot: int, obstacle_links: set[int]) -> bool:
    """Return whether an arm link contacts the mast, base, mount, or neck.

    The robot must be loaded with PyBullet self-collision enabled and the URDF
    must provide collision geometry for the relevant links.
    """
    pb.performCollisionDetection()
    for contact in pb.getContactPoints(bodyA=robot, bodyB=robot):
        link_a, link_b = contact[3], contact[4]
        if (link_a in obstacle_links) != (link_b in obstacle_links):
            return True
    return False


def workspace_samples(
    robot: int,
    sample_count: int,
    voxel_size_m: float,
    front_min_x_m: float = 0.0,
) -> tuple[list[tuple[float, float, float]], dict[str, object]]:
    """Sample safe front-of-robot URDF configurations and return gripper points.

    Samples behind the robot are rejected by default. Configurations contacting
    the modelled mast, base, mount, or neck are also rejected.
    """
    if sample_count < 1:
        raise ValueError("workspace sample count must be positive")
    if voxel_size_m <= 0:
        raise ValueError("workspace voxel size must be positive")
    indices = joint_indices(robot)
    limits = [(pb.getJointInfo(robot, index)[8], pb.getJointInfo(robot, index)[9]) for index in indices]
    if any(lower >= upper for lower, upper in limits):
        raise RuntimeError("all workspace joints need finite lower and upper URDF limits")
    gripper_link = find_link(robot, "gripper")
    obstacle_links = link_indices_by_name(robot, WORKSPACE_OBSTACLE_LINK_NAMES)
    bases = (2, 3, 5, 7, 11, 13)
    voxels: dict[tuple[int, int, int], tuple[float, float, float]] = {}
    rejected_behind = 0
    rejected_collision = 0
    for sample_index in range(1, sample_count + 1):
        values = [lower + (upper - lower) * halton(sample_index, base)
                  for (lower, upper), base in zip(limits, bases)]
        for joint_index, value in zip(indices, values):
            pb.resetJointState(robot, joint_index, value)
        point = gripper_centre(robot, gripper_link)
        if point[0] < front_min_x_m:
            rejected_behind += 1
            continue
        if collides_with_workspace_obstacle(robot, obstacle_links):
            rejected_collision += 1
            continue
        key = tuple(round(value / voxel_size_m) for value in point)
        voxels.setdefault(key, point)
    if not voxels:
        raise RuntimeError("no workspace samples survived front-of-robot and collision filtering")
    points = list(voxels.values())
    bounds = {
        "x_min_m": min(point[0] for point in points),
        "x_max_m": max(point[0] for point in points),
        "y_min_m": min(point[1] for point in points),
        "y_max_m": max(point[1] for point in points),
        "z_min_m": min(point[2] for point in points),
        "z_max_m": max(point[2] for point in points),
    }
    return points, {
        "frame_id": "base_footprint",
        "kind": "collision-filtered_front_workspace_estimate",
        "source": "URDF geometry, joint limits, and modelled collision volumes",
        "sampled_configurations": sample_count,
        "accepted_configurations": sample_count - rejected_behind - rejected_collision,
        "rejected_behind_robot": rejected_behind,
        "rejected_model_collision": rejected_collision,
        "front_min_x_m": front_min_x_m,
        "voxel_size_m": voxel_size_m,
        "unique_voxels": len(points),
        "bounds_m": bounds,
        "joint_limits_rad": dict(zip(WORKSPACE_JOINT_NAMES, limits)),
        "limitations": [
            "No physical validation is implied.",
            "Servo EEPROM ticks have not yet been calibrated to URDF joint angles.",
            "Platform, cable, and any unmodelled collision volumes are not filtered.",
        ],
    }


def write_workspace_ply(path: Path, points: list[tuple[float, float, float]]) -> None:
    """Write a portable point cloud that can be opened in MeshLab or CloudCompare."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as output:
        output.write("ply\nformat ascii 1.0\n")
        output.write(f"element vertex {len(points)}\n")
        output.write("property float x\nproperty float y\nproperty float z\n")
        output.write("property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
        for x, y, z in points:
            output.write(f"{x:.6f} {y:.6f} {z:.6f} 35 190 75\n")


def nearest_workspace_point(
    points: list[tuple[float, float, float]], x_m: float, y_m: float
) -> tuple[float, float, float]:
    """Return the workspace point nearest to a top-down click."""
    if not points:
        raise ValueError("workspace points must not be empty")
    return min(points, key=lambda point: (point[0] - x_m) ** 2 + (point[1] - y_m) ** 2)


def show_workspace_matplotlib(robot: int, points: list[tuple[float, float, float]]) -> None:
    """Show a clickable top-down workspace view without using the OpenGL GUI."""
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    for index in joint_indices(robot):
        pb.resetJointState(robot, index, 0.0)
    pb.performCollisionDetection()

    figure, axis = plt.subplots(figsize=(9, 8))
    xs, ys, zs = zip(*points)
    cloud = axis.scatter(xs, ys, c=[z * 100.0 for z in zs], cmap="turbo", s=8, alpha=0.58,
                         linewidths=0, label="usable gripper-centre workspace")
    colourbar = figure.colorbar(cloud, ax=axis, pad=0.02)
    colourbar.set_label("gripper-centre height (cm)")

    # Simple top-down robot: base rectangle, mast, and a neutral-pose stick arm.
    axis.add_patch(Rectangle((-0.150, -0.1425), 0.300, 0.285,
                             facecolor="#303030", edgecolor="black", alpha=0.85, label="robot base"))
    axis.add_patch(Rectangle((-0.065, -0.020), 0.040, 0.040,
                             facecolor="#aeb6bf", edgecolor="black", label="aluminium mast"))
    arm_links = ("arm_base_mount", "shoulder_servo", "upper_arm", "elbow_servo", "forearm",
                 "wrist_servo", "wrist_roll_link", "gripper")
    arm_xy = []
    for name in arm_links:
        state = pb.getLinkState(robot, find_link(robot, name), computeForwardKinematics=True)
        arm_xy.append((state[4][0], state[4][1]))
    axis.plot(*zip(*arm_xy), color="#ff7f0e", linewidth=5, marker="o", markersize=5,
              label="arm (neutral stick view)")
    axis.annotate("+x forward", (0.32, 0.0), xytext=(0.10, 0.04),
                  arrowprops={"arrowstyle": "->", "color": "crimson"}, color="crimson")

    selected, = axis.plot([], [], marker="x", color="black", markersize=10, markeredgewidth=2,
                          linestyle="none", label="selected reachable point")
    coordinate_label = axis.text(
        0.02, 0.98, "Click the workspace to read the nearest reachable point in cm.",
        transform=axis.transAxes, va="top", ha="left",
        bbox={"facecolor": "white", "alpha": 0.9, "edgecolor": "0.5"},
    )

    def on_click(event) -> None:
        if event.inaxes is not axis or event.xdata is None or event.ydata is None:
            return
        x_m, y_m, z_m = nearest_workspace_point(points, event.xdata, event.ydata)
        selected.set_data([x_m], [y_m])
        coordinate_label.set_text(
            f"Nearest reachable point: x={x_m * 100:+.1f} cm, "
            f"y={y_m * 100:+.1f} cm, z={z_m * 100:+.1f} cm"
        )
        figure.canvas.draw_idle()

    figure.canvas.mpl_connect("button_press_event", on_click)
    axis.set_title("Homer usable arm workspace — click for centimetre coordinates")
    axis.set_xlabel("x: forward (+) / rear (−) metres")
    axis.set_ylabel("y: left (+) / right (−) metres")
    axis.set_aspect("equal", adjustable="box")
    axis.grid(alpha=0.25)
    axis.legend(loc="lower right")
    figure.tight_layout()
    plt.show()


def workspace_envelope_voxels(
    points: list[tuple[float, float, float]], voxel_size_m: float
) -> set[tuple[int, int, int]]:
    """Create a readable, lightly filled voxel envelope from a sampled cloud."""
    occupied = {
        tuple(round(value / voxel_size_m) for value in point)
        for point in points
    }
    # A one-voxel axial dilation joins neighbouring samples into an envelope
    # without pretending that unbounded space is reachable.
    neighbours = ((0, 0, 0), (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0),
                  (0, 0, 1), (0, 0, -1))
    return {
        (x + dx, y + dy, z + dz)
        for x, y, z in occupied
        for dx, dy, dz in neighbours
    }


def height_colour(z: float, z_min: float, z_max: float) -> tuple[float, float, float, float]:
    """Map low-to-high workspace height to blue, cyan, yellow, then red."""
    fraction = 0.5 if z_max == z_min else (z - z_min) / (z_max - z_min)
    fraction = max(0.0, min(1.0, fraction))
    palette = ((0.10, 0.20, 0.90), (0.00, 0.75, 0.85), (0.95, 0.85, 0.05), (0.90, 0.10, 0.05))
    scaled = fraction * (len(palette) - 1)
    index = min(int(scaled), len(palette) - 2)
    blend = scaled - index
    first, second = palette[index], palette[index + 1]
    return tuple(first[channel] + blend * (second[channel] - first[channel]) for channel in range(3)) + (0.72,)


def add_envelope_mesh(voxels: set[tuple[int, int, int]], voxel_size_m: float) -> None:
    """Render only exposed voxel faces, grouped into height-coloured meshes."""
    if not voxels:
        return
    faces = (
        ((1, 0, 0), ((1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1))),
        ((-1, 0, 0), ((0, 0, 0), (0, 0, 1), (0, 1, 1), (0, 1, 0))),
        ((0, 1, 0), ((0, 1, 0), (0, 1, 1), (1, 1, 1), (1, 1, 0))),
        ((0, -1, 0), ((0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1))),
        ((0, 0, 1), ((0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1))),
        ((0, 0, -1), ((0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0))),
    )
    z_min, z_max = min(key[2] for key in voxels), max(key[2] for key in voxels)
    bands: list[tuple[list[tuple[float, float, float]], list[int]]] = [([], []) for _ in range(8)]
    for x, y, z in voxels:
        band = min(7, int(8 * (z - z_min) / max(1, z_max - z_min)))
        vertices, indices = bands[band]
        for neighbour, corners in faces:
            if (x + neighbour[0], y + neighbour[1], z + neighbour[2]) in voxels:
                continue
            base = len(vertices)
            vertices.extend(
                tuple((coordinate + offset) * voxel_size_m for coordinate, offset in zip((x, y, z), corner))
                for corner in corners
            )
            indices.extend((base, base + 1, base + 2, base, base + 2, base + 3))
    for band, (vertices, indices) in enumerate(bands):
        if not vertices:
            continue
        z = (z_min + (band + 0.5) * (z_max - z_min + 1) / 8) * voxel_size_m
        visual = pb.createVisualShape(
            pb.GEOM_MESH, vertices=vertices, indices=indices,
            rgbaColor=height_colour(z, z_min * voxel_size_m, z_max * voxel_size_m),
        )
        pb.createMultiBody(baseMass=0, baseVisualShapeIndex=visual)


def add_envelope_contours(voxels: set[tuple[int, int, int]], voxel_size_m: float) -> None:
    """Render thin horizontal contour bands so the robot stays visible."""
    if not voxels:
        return
    z_min, z_max = min(key[2] for key in voxels), max(key[2] for key in voxels)
    visuals = []
    for band in range(8):
        z = (z_min + (band + 0.5) * (z_max - z_min + 1) / 8) * voxel_size_m
        visuals.append(pb.createVisualShape(
            pb.GEOM_BOX,
            halfExtents=(voxel_size_m * 0.48, voxel_size_m * 0.48, 0.0025),
            rgbaColor=height_colour(z, z_min * voxel_size_m, z_max * voxel_size_m),
        ))
    for x, y, z in voxels:
        # A 2-D boundary at each height creates readable contour rings rather
        # than an opaque solid that hides the robot inside it.
        if all((x + dx, y + dy, z) in voxels for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))):
            continue
        band = min(7, int(8 * (z - z_min) / max(1, z_max - z_min)))
        pb.createMultiBody(
            baseMass=0, baseVisualShapeIndex=visuals[band],
            basePosition=(x * voxel_size_m, y * voxel_size_m, z * voxel_size_m),
        )


def add_workspace_slice(
    points: list[tuple[float, float, float]], slice_z_m: float, voxel_size_m: float
) -> int:
    """Render a thin orange floor slice through the reachable cloud."""
    cells = {
        (round(x / voxel_size_m), round(y / voxel_size_m))
        for x, y, z in points
        if abs(z - slice_z_m) <= voxel_size_m
    }
    visual = pb.createVisualShape(
        pb.GEOM_BOX, halfExtents=(voxel_size_m * 0.46, voxel_size_m * 0.46, 0.002),
        rgbaColor=(1.0, 0.45, 0.0, 0.92),
    )
    for x, y in cells:
        pb.createMultiBody(baseMass=0, baseVisualShapeIndex=visual,
                           basePosition=(x * voxel_size_m, y * voxel_size_m, slice_z_m))
    return len(cells)


def show_workspace(
    points: list[tuple[float, float, float]],
    envelope_voxel_m: float,
    slice_z_m: float | None,
) -> None:
    """Render a height-coloured workspace envelope and optional platform slice."""
    envelope = workspace_envelope_voxels(points, envelope_voxel_m)
    add_envelope_contours(envelope, envelope_voxel_m)
    if slice_z_m is not None:
        slice_cells = add_workspace_slice(points, slice_z_m, envelope_voxel_m)
        pb.addUserDebugText(
            f"workspace slice z={slice_z_m:.3f} m ({slice_cells} cells)",
            (0.0, 0.0, slice_z_m + 0.03), textColorRGB=(1, 0.35, 0), textSize=1.3,
        )
    for axis, endpoint, colour in (
        ("+x forward", (0.30, 0.0, 0.0), (1, 0, 0)),
        ("+y left", (0.0, 0.30, 0.0), (0, 1, 0)),
        ("+z up", (0.0, 0.0, 0.30), (0, 0, 1)),
    ):
        pb.addUserDebugLine((0, 0, 0), endpoint, lineColorRGB=colour, lineWidth=3)
        pb.addUserDebugText(axis, endpoint, textColorRGB=colour, textSize=1.2)
    pb.resetDebugVisualizerCamera(1.4, 45, -24, (0.0, 0.0, 0.30))


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


def add_lidar_noise(ranges: list[float], rng: random.Random, std_m: float) -> list[float]:
    """Apply range noise while preserving the sensor's valid range."""
    if std_m <= 0.0:
        return ranges
    return [max(0.0, min(LIDAR_RANGE_M, distance + rng.gauss(0.0, std_m)))
            for distance in ranges]


def integrate_wheel_odometry(
    pose: tuple[float, float, float],
    left_distance: float,
    right_distance: float,
) -> tuple[float, float, float]:
    """Integrate measured differential-drive wheel travel into an odom pose."""
    x, y, theta = pose
    center_distance = (left_distance + right_distance) / 2.0
    heading_change = (right_distance - left_distance) / TRACK_WIDTH_M
    mid_heading = theta + heading_change / 2.0
    return (
        x + center_distance * math.cos(mid_heading),
        y + center_distance * math.sin(mid_heading),
        theta + heading_change,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=Path(os.environ.get("ROBOT_URDF", DEFAULT_URDF)))
    parser.add_argument("--gui", action="store_true", help="Open the PyBullet window")
    parser.add_argument(
        "--workspace",
        action="store_true",
        help="Sample the collision-filtered, front-of-robot gripper workspace estimate",
    )
    parser.add_argument(
        "--workspace-matplotlib",
        action="store_true",
        help="Open a clickable Matplotlib top-down workspace view instead of the PyBullet OpenGL view",
    )
    parser.add_argument(
        "--workspace-front-min-x-m",
        type=float,
        default=0.0,
        help="Keep only gripper-centre points at or in front of this base_footprint x coordinate (default: 0)",
    )
    parser.add_argument(
        "--workspace-samples",
        type=int,
        default=12000,
        help="Number of deterministic joint configurations to sample with --workspace",
    )
    parser.add_argument(
        "--workspace-voxel-m",
        type=float,
        default=0.01,
        help="Voxel size used to deduplicate workspace points with --workspace",
    )
    parser.add_argument(
        "--workspace-envelope-voxel-m",
        type=float,
        default=0.03,
        help="Coarser voxel size used for the readable workspace envelope in the GUI",
    )
    parser.add_argument(
        "--workspace-slice-z-m",
        type=float,
        help="Optional measured platform height for an orange reachable-area slice",
    )
    parser.add_argument(
        "--workspace-output",
        type=Path,
        default=Path("data/reachability/urdf_workspace.ply"),
        help="PLY point-cloud output path for --workspace",
    )
    parser.add_argument("--2d-gui", dest="view_2d_gui", action="store_true",
                        help="Open a realtime PyBullet top-down 2D-like view")
    parser.add_argument("--2d-live", dest="view_2d_live", action="store_true",
                        help="Open a genuine realtime flat 2D window")
    parser.add_argument("--2d", dest="view_2d", action="store_true",
                        help="Open a lightweight top-down scan view")
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--seed", type=int, default=7,
                        help="Random seed for repeatable sensor errors")
    parser.add_argument("--lidar-noise-std", type=float, default=LIDAR_NOISE_STD_M,
                        help="LiDAR Gaussian range noise standard deviation in metres")
    parser.add_argument("--wheel-noise-fraction", type=float, default=WHEEL_NOISE_FRACTION,
                        help="Per-step wheel travel noise as a fraction of travel")
    parser.add_argument("--wheel-scale-error", type=float, default=WHEEL_SCALE_ERROR,
                        help="Fixed left/right wheel scale error fraction")
    parser.add_argument("--save-2d", type=Path, metavar="PNG",
                        help="Save a final 2D view image (useful on headless systems)")
    args = parser.parse_args()
    if args.workspace_matplotlib and not args.workspace:
        parser.error("--workspace-matplotlib requires --workspace")
    if not args.urdf.exists():
        raise SystemExit(f"URDF not found: {args.urdf}")

    view = make_2d_view() if args.view_2d else None
    rng = random.Random(args.seed)
    # Fixed calibration errors model unequal wheel diameter/encoder scale.
    left_scale = 1.0 + rng.uniform(-args.wheel_scale_error, args.wheel_scale_error)
    right_scale = 1.0 + rng.uniform(-args.wheel_scale_error, args.wheel_scale_error)
    use_2d_gui = args.view_2d_gui
    live_2d_view = make_live_2d_view() if args.view_2d_live else None
    client = pb.connect(pb.GUI if args.gui or use_2d_gui else pb.DIRECT)
    try:
        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        if args.workspace:
            # Fixed base keeps all output coordinates in the URDF's
            # base_footprint frame rather than a moving simulation world frame.
            robot = pb.loadURDF(
                str(args.urdf), useFixedBase=True,
                flags=(pb.URDF_USE_INERTIA_FROM_FILE | pb.URDF_USE_SELF_COLLISION |
                       pb.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT),
            )
            points, summary = workspace_samples(
                robot, args.workspace_samples, args.workspace_voxel_m, args.workspace_front_min_x_m
            )
            write_workspace_ply(args.workspace_output, points)
            print(json.dumps(summary, indent=2, sort_keys=True))
            print(f"wrote {len(points)} workspace points to {args.workspace_output}")
            if args.workspace_matplotlib:
                show_workspace_matplotlib(robot, points)
                return
            if args.gui:
                show_workspace(
                    points, args.workspace_envelope_voxel_m, args.workspace_slice_z_m
                )
                print("Height-coloured surface is the collision-filtered front workspace estimate; close the GUI to exit.")
                started = time.monotonic()
                while time.monotonic() - started < args.seconds:
                    pb.stepSimulation()
                    time.sleep(1.0 / 60.0)
            return
        pb.setGravity(0, 0, -9.81)
        pb.loadURDF("plane.urdf")
        make_test_world()
        robot = pb.loadURDF(str(args.urdf), basePosition=(0, 0, 0.02), useFixedBase=False,
                            flags=pb.URDF_USE_INERTIA_FROM_FILE)
        pybullet_2d_view = make_pybullet_2d_view() if use_2d_gui else None
        lidar_link = find_link(robot, "rplidar_a1")

        x, y, theta = 0.0, 0.0, 0.0       # ground truth pose
        odom_pose = (0.0, 0.0, 0.0)       # pose reconstructed from wheel readings
        started = time.monotonic()
        last = started
        sample = 0
        while time.monotonic() - started < args.seconds:
            now = time.monotonic()
            dt = min(now - last, 0.05)
            last = now
            phase = (now - started) % 16.0
            linear, angular = (0.10, 0.0) if phase < 8.0 else (0.0, math.pi / 8.0)
            left_velocity = linear - angular * TRACK_WIDTH_M / 2.0
            right_velocity = linear + angular * TRACK_WIDTH_M / 2.0
            left_distance = left_velocity * dt
            right_distance = right_velocity * dt
            x += linear * math.cos(theta) * dt
            y += linear * math.sin(theta) * dt
            theta += angular * dt
            left_measured = left_distance * left_scale
            right_measured = right_distance * right_scale
            left_measured += rng.gauss(0.0, abs(left_distance) * args.wheel_noise_fraction)
            right_measured += rng.gauss(0.0, abs(right_distance) * args.wheel_noise_fraction)
            odom_pose = integrate_wheel_odometry(odom_pose, left_measured, right_measured)
            pb.resetBasePositionAndOrientation(robot, (x, y, 0.02), pb.getQuaternionFromEuler((0, 0, theta)))
            ranges = add_lidar_noise(scan(robot, lidar_link, (x, y, theta)), rng,
                                     args.lidar_noise_std)
            if live_2d_view and not update_live_2d_view(live_2d_view, odom_pose, ranges):
                break
            if pybullet_2d_view:
                update_pybullet_2d_view(pybullet_2d_view, odom_pose, ranges)
            if view and sample % 3 == 0:
                update_2d_view(view, odom_pose, ranges)
            if sample % 20 == 0:
                position_error = math.hypot(odom_pose[0] - x, odom_pose[1] - y)
                print(f"t={now-started:5.1f}s true=({x:+.3f}, {y:+.3f}, {theta:+.3f}) "
                      f"odom=({odom_pose[0]:+.3f}, {odom_pose[1]:+.3f}, {odom_pose[2]:+.3f}) "
                      f"odom_xy_error={position_error:.3f}m scan_min={min(ranges):.3f}m")
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
