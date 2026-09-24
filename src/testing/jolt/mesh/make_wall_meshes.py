#!/usr/bin/env python3
"""Generate wall, ramp and step fixtures for ground-robot traction tests."""

from pathlib import Path
import math
import sys

from make_meshes import Mesh, write_glb

for name, lip_height in (
    ("wall_plain.glb", 0),
    ("wall_lip.glb", 0.03),
    ("wall_high_lip.glb", 0.10),
):
    mesh = Mesh()
    mesh.add_quad((-5, -20, 0), (5, -20, 0), (5, 20, 0), (-5, 20, 0))
    mesh.add_box((1, -20, 0), (2, 20, 3))
    if lip_height:
        mesh.add_box((0.85, -20, 0), (1, 20, lip_height))
    write_glb(str(Path(sys.argv[1]) / name), [(mesh, (0, 0, 0))])

mesh = Mesh()
mesh.add_quad((-10, -10, 0), (20, -10, 0), (20, 10, 0), (-10, 10, 0))
mesh.add_box((-10, -10, 0), (1, 10, 1))
write_glb(str(Path(sys.argv[1]) / "ledge.glb"), [(mesh, (0, 0, 0))])

for angle in (16, 18):
    slope = math.tan(math.radians(angle))
    mesh = Mesh()
    mesh.add_quad(
        (-5, -10, -5 * slope),
        (20, -10, 20 * slope),
        (20, 10, 20 * slope),
        (-5, 10, -5 * slope),
    )
    write_glb(str(Path(sys.argv[1]) / f"ramp{angle}.glb"), [(mesh, (0, 0, 0))])

for centimetres in (10, 20, 30, 35, 40):
    mesh = Mesh()
    mesh.add_quad((-5, -10, 0), (20, -10, 0), (20, 10, 0), (-5, 10, 0))
    mesh.add_box((1, -10, 0), (10, 10, centimetres / 100))
    write_glb(str(Path(sys.argv[1]) / f"step{centimetres}.glb"), [(mesh, (0, 0, 0))])
