#!/usr/bin/env python3
"""Generate a wide wall, with and without a 3 cm toe, for traction tests."""

from pathlib import Path
import sys

from make_meshes import Mesh, write_glb

for lip in (False, True):
    mesh = Mesh()
    mesh.add_quad((-5, -20, 0), (5, -20, 0), (5, 20, 0), (-5, 20, 0))
    mesh.add_box((1, -20, 0), (2, 20, 3))
    if lip:
        mesh.add_box((0.85, -20, 0), (1, 20, 0.03))
    name = "wall_lip.glb" if lip else "wall_plain.glb"
    write_glb(str(Path(sys.argv[1]) / name), [(mesh, (0, 0, 0))])
