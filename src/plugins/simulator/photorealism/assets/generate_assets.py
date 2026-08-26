#!/usr/bin/env python3
"""Generates the glTF visual assets of the photorealism plugin.

The models are authored in the ARGoS frame (z up, meters, origin at
the entity's origin anchor) and rotated to the glTF convention (y up)
on export; the .visual.xml descriptors rotate them back with
orientation="0,0,90".

Usage (any environment with trimesh + numpy):

    python3 -m venv /tmp/assets-venv
    /tmp/assets-venv/bin/pip install trimesh numpy pillow
    /tmp/assets-venv/bin/python generate_assets.py

The foot-bot is procedural (dimensions from qtopengl_footbot.cpp);
the drone converts the OBJ models shipped with the drone plugin. The
Scout Mini and Spot are procedural too, with their dimensions taken
from the matching entity plugins so that the geometry a camera sees
and the geometry a robot collides with cannot drift apart.
"""

import os
import numpy as np
import trimesh
from trimesh.visual.material import PBRMaterial

HERE = os.path.dirname(os.path.abspath(__file__))
DRONE_MODELS = os.path.normpath(os.path.join(
    HERE, "../../../robots/drone/simulator/models"))

# Rotates the ARGoS frame (z up) onto the glTF frame (y up)
Z_UP_TO_Y_UP = trimesh.transformations.rotation_matrix(
    -np.pi / 2.0, [1, 0, 0])


def material(name, rgb, roughness=0.7, metallic=0.0):
    return PBRMaterial(name=name,
                       baseColorFactor=list(rgb) + [1.0],
                       roughnessFactor=roughness,
                       metallicFactor=metallic)


def part(mesh, transform, mat):
    mesh.apply_transform(transform)
    mesh.visual = trimesh.visual.TextureVisuals(material=mat)
    return mesh


def translate(x, y, z):
    return trimesh.transformations.translation_matrix([x, y, z])


def rot_y_axis():
    """Cylinder axis from z onto y (for the wheels)."""
    return trimesh.transformations.rotation_matrix(np.pi / 2.0, [1, 0, 0])


def rot_x_axis():
    """Cylinder axis from z onto x (for masts and lens barrels)."""
    return trimesh.transformations.rotation_matrix(np.pi / 2.0, [0, 1, 0])


def box(sx, sy, sz):
    return trimesh.creation.box(extents=[sx, sy, sz])


def make_footbot():
    """Procedural foot-bot, dimensions from qtopengl_footbot.cpp."""
    BODY_RADIUS = 0.085036758
    TURRET_RADIUS = 0.069
    WHEEL_RADIUS = 0.029112741
    WHEEL_WIDTH = 0.022031354
    HALF_INTERWHEEL = 0.0635
    DARK = material("dark", (0.08, 0.08, 0.09), roughness=0.8)
    BODY = material("body", (0.30, 0.32, 0.35), roughness=0.55, metallic=0.4)
    RED = material("turret", (0.72, 0.06, 0.06), roughness=0.45)
    GRAY = material("gray", (0.55, 0.55, 0.58), roughness=0.6, metallic=0.3)
    parts = []
    # wheels (cylinder axis along y)
    for side in (-1.0, 1.0):
        parts.append(part(
            trimesh.creation.cylinder(radius=WHEEL_RADIUS,
                                      height=WHEEL_WIDTH, sections=24),
            translate(0, side * HALF_INTERWHEEL, WHEEL_RADIUS) @ rot_y_axis(),
            DARK))
    # chassis
    parts.append(part(
        trimesh.creation.cylinder(radius=BODY_RADIUS, height=0.045,
                                  sections=32),
        translate(0, 0, 0.0375), BODY))
    # proximity sensor ring
    parts.append(part(
        trimesh.creation.cylinder(radius=BODY_RADIUS + 0.002, height=0.018,
                                  sections=32),
        translate(0, 0, 0.069), DARK))
    # gripper turret
    parts.append(part(
        trimesh.creation.cylinder(radius=TURRET_RADIUS, height=0.04,
                                  sections=32),
        translate(0, 0, 0.098), RED))
    # distance scanner
    parts.append(part(
        trimesh.creation.cylinder(radius=0.045, height=0.016, sections=24),
        translate(0, 0, 0.126), DARK))
    # camera pole
    parts.append(part(
        trimesh.creation.cylinder(radius=0.006, height=0.104, sections=12),
        translate(0, 0, 0.186), GRAY))
    # omnidirectional camera housing
    parts.append(part(
        trimesh.creation.cylinder(radius=0.02, height=0.02, sections=16),
        translate(0, 0, 0.248), DARK))
    scene = trimesh.Scene()
    for i, p in enumerate(parts):
        p.apply_transform(Z_UP_TO_Y_UP)
        scene.add_geometry(p, node_name=f"part_{i}")
    return scene


def make_drone():
    """The drone OBJ models shipped with the drone plugin (z up,
    meters), with the four propellers placed like qtopengl_drone.cpp
    does."""
    scene = trimesh.Scene()
    body = trimesh.load(os.path.join(DRONE_MODELS, "drone.obj"))
    for name, geom in body.geometry.items():
        g = geom.copy()
        g.apply_transform(Z_UP_TO_Y_UP)
        scene.add_geometry(g, node_name=f"body_{name}")
    prop = trimesh.load(os.path.join(DRONE_MODELS, "propeller.obj"),
                        force='mesh')
    OFFSET = (0.159, 0.159, 0.271)  # CQTOpenGLDrone::m_cPropellerOffset
    for i, (sx, sy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1))):
        p = prop.copy()
        p.apply_transform(translate(sx * OFFSET[0], sy * OFFSET[1], OFFSET[2]))
        p.apply_transform(Z_UP_TO_Y_UP)
        p.visual = trimesh.visual.TextureVisuals(
            material=material("propeller", (0.10, 0.10, 0.11), roughness=0.5))
        scene.add_geometry(p, node_name=f"propeller_{i}")
    return scene


def make_scout_mini():
    """Procedural AgileX Scout Mini, dimensions from the entity plugin.

    Authored in the ARGoS frame with the origin anchor on the floor, so every
    number here is the same absolute height the entity's anchors and
    SwarmDeck's RobotSpec use: wheel centre 0.0875, deck top 0.2435, mapping
    lidar 0.4525, camera 0.2125.
    """
    WHEEL_RADIUS = 0.0875
    WHEEL_WIDTH = 0.070
    HALF_TRACK = 0.225           # SCOUT_MINI_TRACK_GAUGE 0.450
    HALF_WHEELBASE = 0.203
    BLACK = material("scout_black", (0.06, 0.06, 0.07), roughness=0.75)
    ORANGE = material("scout_orange", (0.85, 0.35, 0.05), roughness=0.45)
    METAL = material("scout_metal", (0.55, 0.56, 0.60), roughness=0.35,
                     metallic=0.7)
    GLASS = material("scout_lens", (0.05, 0.06, 0.10), roughness=0.15,
                     metallic=0.2)
    parts = []
    # Four driven wheels (cylinder axis along y)
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            parts.append(part(
                trimesh.creation.cylinder(radius=WHEEL_RADIUS,
                                          height=WHEEL_WIDTH, sections=28),
                translate(sx * HALF_WHEELBASE, sy * HALF_TRACK, WHEEL_RADIUS)
                @ rot_y_axis(),
                BLACK))
    # Chassis and deck plate: together they reach the 0.2435 deck top.
    parts.append(part(box(0.520, 0.440, 0.140), translate(0, 0, 0.155),
                      ORANGE))
    parts.append(part(box(0.560, 0.500, 0.020), translate(0, 0, 0.2335),
                      BLACK))
    # Lidar mast and puck at the mapping lidar mount (-0.080, 0, 0.4525)
    parts.append(part(
        trimesh.creation.cylinder(radius=0.022, height=0.175, sections=16),
        translate(-0.080, 0, 0.331), METAL))
    parts.append(part(
        trimesh.creation.cylinder(radius=0.052, height=0.072, sections=28),
        translate(-0.080, 0, 0.4525), BLACK))
    # Forward camera housing at (0.322, 0, 0.2125)
    parts.append(part(box(0.036, 0.110, 0.042), translate(0.300, 0, 0.2125),
                      BLACK))
    parts.append(part(
        trimesh.creation.cylinder(radius=0.014, height=0.014, sections=16),
        translate(0.322, 0, 0.2125) @ rot_x_axis(), GLASS))
    scene = trimesh.Scene()
    for i, mesh in enumerate(parts):
        mesh.apply_transform(Z_UP_TO_Y_UP)
        scene.add_geometry(mesh, node_name=f"part_{i}")
    return scene


def make_bunker():
    """Procedural AgileX Bunker (full size), dimensions from the entity plugin.

    Not to be confused with the bunker_mini built into pr_scene_sync: this is
    the 1.023 x 0.778 m, 170 kg machine SwarmDeck's hardware fleet actually
    runs. Heights are absolute, measured from the origin anchor on the floor,
    so they match the entity's anchors and SwarmDeck's RobotSpec: deck top
    0.380, mapping lidar 0.720, camera 0.300.
    """
    TRACK_LENGTH = 1.023
    TRACK_WIDTH = 0.158
    TRACK_HEIGHT = 0.200
    HALF_TRACK_GAUGE = 0.310
    BLACK = material("bunker_track", (0.05, 0.05, 0.06), roughness=0.85)
    ORANGE = material("bunker_hull", (0.86, 0.36, 0.05), roughness=0.45)
    DARK = material("bunker_dark", (0.10, 0.10, 0.11), roughness=0.60)
    METAL = material("bunker_metal", (0.55, 0.56, 0.60), roughness=0.35,
                     metallic=0.7)
    GLASS = material("bunker_lens", (0.05, 0.06, 0.10), roughness=0.15,
                     metallic=0.2)
    parts = []
    # Tracks, as slab boxes with rounded sprockets at each end.
    for side in (-1.0, 1.0):
        parts.append(part(box(TRACK_LENGTH, TRACK_WIDTH, TRACK_HEIGHT),
                          translate(0, side * HALF_TRACK_GAUGE,
                                    TRACK_HEIGHT * 0.5),
                          BLACK))
        for end in (-1.0, 1.0):
            parts.append(part(
                trimesh.creation.cylinder(radius=0.100, height=TRACK_WIDTH,
                                          sections=24),
                translate(end * (TRACK_LENGTH * 0.5 - 0.10),
                          side * HALF_TRACK_GAUGE, 0.100) @ rot_y_axis(),
                DARK))
    # Hull between the tracks, then the deck plate at 0.380.
    parts.append(part(box(0.880, 0.470, 0.180), translate(0, 0, 0.280),
                      ORANGE))
    parts.append(part(box(0.940, 0.700, 0.020), translate(0, 0, 0.370),
                      DARK))
    # Lidar mast and puck at (-0.150, 0, 0.720)
    parts.append(part(
        trimesh.creation.cylinder(radius=0.026, height=0.310, sections=16),
        translate(-0.150, 0, 0.535), METAL))
    parts.append(part(
        trimesh.creation.cylinder(radius=0.052, height=0.072, sections=28),
        translate(-0.150, 0, 0.720), BLACK))
    # Forward camera at (0.515, 0, 0.300)
    parts.append(part(box(0.040, 0.130, 0.048), translate(0.492, 0, 0.300),
                      DARK))
    parts.append(part(
        trimesh.creation.cylinder(radius=0.016, height=0.016, sections=16),
        translate(0.515, 0, 0.300) @ rot_x_axis(), GLASS))
    scene = trimesh.Scene()
    for i, mesh in enumerate(parts):
        mesh.apply_transform(Z_UP_TO_Y_UP)
        scene.add_geometry(mesh, node_name=f"part_{i}")
    return scene


def make_spot():
    """Procedural Boston Dynamics Spot, dimensions from the entity plugin.

    The legs are why this is a model rather than a box. The Jolt collision
    shape is the whole standing envelope, but the cameras and the
    photorealistic lidar raytrace THIS geometry, so a neighbour sees a body
    carried on four legs with daylight between them, which is what a real
    Spot looks like to a range sensor.
    """
    BODY_TOP = 0.620             # SPOT_HEIGHT, the deck top
    BODY_BOTTOM = 0.430
    HIP_Z = 0.470
    HALF_LEN = 0.360
    HALF_WIDTH = 0.180
    YELLOW = material("spot_yellow", (0.79, 0.63, 0.00), roughness=0.40)
    BLACK = material("spot_black", (0.07, 0.07, 0.08), roughness=0.65)
    METAL = material("spot_metal", (0.50, 0.51, 0.55), roughness=0.35,
                     metallic=0.7)
    GLASS = material("spot_lens", (0.05, 0.06, 0.10), roughness=0.15,
                     metallic=0.2)
    parts = []
    # Hull
    parts.append(part(box(1.020, 0.440, BODY_TOP - BODY_BOTTOM),
                      translate(0, 0, (BODY_TOP + BODY_BOTTOM) * 0.5),
                      YELLOW))
    # Front sensor head and rear battery pack
    parts.append(part(box(0.090, 0.360, 0.130), translate(0.500, 0, 0.520),
                      BLACK))
    parts.append(part(box(0.120, 0.380, 0.110), translate(-0.430, 0, 0.540),
                      BLACK))
    # Four legs: a thigh angled outwards, then a shank down to the floor.
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            hip = (sx * HALF_LEN, sy * HALF_WIDTH, HIP_Z)
            knee = (sx * (HALF_LEN + 0.055), sy * 0.235, 0.245)
            foot = (sx * HALF_LEN, sy * 0.205, 0.0)
            parts.append(part(
                trimesh.creation.cylinder(radius=0.038, height=0.070,
                                          sections=16),
                translate(*hip) @ rot_y_axis(), YELLOW))
            for start, end, radius in ((hip, knee, 0.032),
                                       (knee, foot, 0.026)):
                parts.append(part(
                    trimesh.creation.cylinder(segment=[list(start), list(end)],
                                              radius=radius, sections=14),
                    np.eye(4), BLACK))
    # Mapping lidar on its mast, at (-0.180, 0, 0.970)
    parts.append(part(
        trimesh.creation.cylinder(radius=0.024, height=0.315, sections=16),
        translate(-0.180, 0, 0.778), METAL))
    parts.append(part(
        trimesh.creation.cylinder(radius=0.052, height=0.072, sections=28),
        translate(-0.180, 0, 0.970), BLACK))
    # Forward camera at (0.598, 0, 0.520)
    parts.append(part(
        trimesh.creation.cylinder(radius=0.016, height=0.016, sections=16),
        translate(0.598, 0, 0.520) @ rot_x_axis(), GLASS))
    scene = trimesh.Scene()
    for i, mesh in enumerate(parts):
        mesh.apply_transform(Z_UP_TO_Y_UP)
        scene.add_geometry(mesh, node_name=f"part_{i}")
    return scene


def export(scene, name):
    path = os.path.join(HERE, name)
    scene.export(path)
    print(f"{name}: {os.path.getsize(path) / 1024:.0f} KiB")


if __name__ == "__main__":
    export(make_footbot(), "foot-bot.glb")
    export(make_drone(), "drone.glb")
    export(make_scout_mini(), "scout_mini.glb")
    export(make_bunker(), "bunker.glb")
    export(make_spot(), "spot.glb")
