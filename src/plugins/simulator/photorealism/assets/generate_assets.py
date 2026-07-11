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
the drone converts the OBJ models shipped with the drone plugin.
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


def export(scene, name):
    path = os.path.join(HERE, name)
    scene.export(path)
    print(f"{name}: {os.path.getsize(path) / 1024:.0f} KiB")


if __name__ == "__main__":
    export(make_footbot(), "foot-bot.glb")
    export(make_drone(), "drone.glb")
