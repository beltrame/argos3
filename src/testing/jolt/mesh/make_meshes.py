#!/usr/bin/env python3
#
# Generates the glTF assets the Jolt mesh tests run against.
#
# Only the Python standard library is used: the .glb container is written
# directly (JSON chunk plus binary chunk), which is all a glTF reader needs
# and keeps the tests reproducible without trimesh or pygltflib.
#
# Geometry is authored in ARGoS coordinates (Z up) and written out Y-up, the
# convention the glTF specification mandates and the one the <mesh> entity
# expects with its default y_up="true". The conversion applied here is the
# inverse of the loader's:
#
#     (X, Y, Z)_ARGoS  ->  (X, Z, -Y)_glTF
#
# Triangles are wound counter-clockwise seen from the side the surface normal
# points at. In the corridor every normal points into the free space, so the
# asset is also correct for entities that turn double_sided off.
#
# Assets written into the directory given as the only argument:
#
#   corridor.glb      an open-ended corridor with a window opening in one wall,
#                     a floor that turns into a 1:4 ramp and then a landing.
#                     The ramp and the landing sit in a child node with a
#                     translation, so a reader that ignored node transforms
#                     would answer the sloped-surface rays wrongly.
#   flipped_wall.glb  one wall across the corridor landing whose triangles are
#                     wound the wrong way, so its normal points away from an
#                     approaching robot.
#   tiled_floor.glb   a flat floor tessellated into 2 m cells, laid out so
#                     that a robot driving along the X axis crosses about 30
#                     internal edges of the triangulation rather than running
#                     along one of them.
#   terrain.glb       an undulating 130 x 100 m terrain with perimeter walls
#                     and scattered blocks, about 105,000 triangles.
#
# @author lemonci - <monica.li@outlook.com>

import json
import math
import os
import struct
import sys

# ---------------------------------------------------------------- mesh builder


class Mesh:
    """Vertex and index accumulator, in ARGoS coordinates."""

    def __init__(self):
        self.vertices = []
        self.indices = []

    def add_quad(self, p0, p1, p2, p3):
        """Adds two triangles; the normal follows the right-hand rule on
        (p1 - p0) x (p2 - p0)."""
        base = len(self.vertices)
        self.vertices.extend([p0, p1, p2, p3])
        self.indices.extend([base, base + 1, base + 2,
                             base, base + 2, base + 3])

    def add_grid(self, corner, du, dv, nu, nv, height=None):
        """Adds a tessellated parallelogram. `corner` is the origin, `du` and
        `dv` the per-cell steps, `nu` and `nv` the cell counts. `height(x, y)`
        optionally displaces every vertex along Z. The normal points along
        du x dv."""
        base = len(self.vertices)
        for j in range(nv + 1):
            for i in range(nu + 1):
                p = [corner[k] + du[k] * i + dv[k] * j for k in range(3)]
                if height is not None:
                    p[2] += height(p[0], p[1])
                self.vertices.append(tuple(p))
        row = nu + 1
        for j in range(nv):
            for i in range(nu):
                a = base + j * row + i
                b = a + 1
                c = a + row + 1
                d = a + row
                self.indices.extend([a, b, c, a, c, d])

    def add_box(self, lo, hi):
        """Adds a closed axis-aligned box whose normals point outward."""
        x0, y0, z0 = lo
        x1, y1, z1 = hi
        faces = [
            # -Z, +Z
            [(x0, y1, z0), (x1, y1, z0), (x1, y0, z0), (x0, y0, z0)],
            [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)],
            # -Y, +Y
            [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)],
            [(x1, y1, z0), (x0, y1, z0), (x0, y1, z1), (x1, y1, z1)],
            # -X, +X
            [(x0, y1, z0), (x0, y0, z0), (x0, y0, z1), (x0, y1, z1)],
            [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)],
        ]
        for f in faces:
            self.add_quad(*f)

    def flip(self):
        """Reverses the winding of every triangle, i.e. flips every normal."""
        for t in range(0, len(self.indices), 3):
            self.indices[t + 1], self.indices[t + 2] = \
                self.indices[t + 2], self.indices[t + 1]

    def triangles(self):
        return len(self.indices) // 3


def wall_with_hole(mesh, y, x_range, z_range, hole_x, hole_z, normal_minus_y):
    """Adds a wall on the plane Y = y with a rectangular opening in it."""
    x0, x1 = x_range
    z0, z1 = z_range
    hx0, hx1 = hole_x
    hz0, hz1 = hole_z
    strips = [
        ((x0, x1), (z0, hz0)),      # below the opening
        ((x0, x1), (hz1, z1)),      # above the opening
        ((x0, hx0), (hz0, hz1)),    # to one side of it
        ((hx1, x1), (hz0, hz1)),    # to the other
    ]
    for (a, b), (c, d) in strips:
        if b <= a or d <= c:
            continue
        p = [(a, y, c), (b, y, c), (b, y, d), (a, y, d)]
        # This winding gives -Y; reverse it for +Y
        if normal_minus_y:
            mesh.add_quad(*p)
        else:
            mesh.add_quad(p[0], p[3], p[2], p[1])


# ------------------------------------------------------------------ glb writer


def write_glb(path, nodes):
    """Writes one .glb. `nodes` is a list of (Mesh, translation_argos); each
    entry becomes one glTF node holding one mesh, and the translation is
    written in glTF axes."""
    buf = bytearray()
    accessors = []
    buffer_views = []
    meshes = []
    gltf_nodes = []

    for mesh, translation in nodes:
        # POSITION, ARGoS (X, Y, Z) -> glTF (X, Z, -Y)
        while len(buf) % 4:
            buf.append(0)
        pos_offset = len(buf)
        mins = [math.inf] * 3
        maxs = [-math.inf] * 3
        for (x, y, z) in mesh.vertices:
            g = (x, z, -y)
            buf.extend(struct.pack("<3f", *g))
            for k in range(3):
                mins[k] = min(mins[k], g[k])
                maxs[k] = max(maxs[k], g[k])
        buffer_views.append({"buffer": 0, "byteOffset": pos_offset,
                             "byteLength": len(buf) - pos_offset,
                             "target": 34962})
        accessors.append({"bufferView": len(buffer_views) - 1,
                          "componentType": 5126, "count": len(mesh.vertices),
                          "type": "VEC3", "min": mins, "max": maxs})
        pos_accessor = len(accessors) - 1

        while len(buf) % 4:
            buf.append(0)
        idx_offset = len(buf)
        buf.extend(struct.pack("<%dI" % len(mesh.indices), *mesh.indices))
        buffer_views.append({"buffer": 0, "byteOffset": idx_offset,
                             "byteLength": len(buf) - idx_offset,
                             "target": 34963})
        accessors.append({"bufferView": len(buffer_views) - 1,
                          "componentType": 5125, "count": len(mesh.indices),
                          "type": "SCALAR"})
        idx_accessor = len(accessors) - 1

        meshes.append({"primitives": [{"attributes": {"POSITION": pos_accessor},
                                       "indices": idx_accessor, "mode": 4}]})
        node = {"mesh": len(meshes) - 1}
        if translation != (0.0, 0.0, 0.0):
            tx, ty, tz = translation
            node["translation"] = [tx, tz, -ty]
        gltf_nodes.append(node)

    gltf = {
        "asset": {"version": "2.0", "generator": "make_meshes.py"},
        "scene": 0,
        "scenes": [{"nodes": list(range(len(gltf_nodes)))}],
        "nodes": gltf_nodes,
        "meshes": meshes,
        "accessors": accessors,
        "bufferViews": buffer_views,
        "buffers": [{"byteLength": len(buf)}],
    }
    js = json.dumps(gltf, separators=(",", ":")).encode("utf-8")
    js += b" " * ((4 - len(js) % 4) % 4)
    bin_chunk = bytes(buf) + b"\0" * ((4 - len(buf) % 4) % 4)
    total = 12 + 8 + len(js) + 8 + len(bin_chunk)
    with open(path, "wb") as f:
        f.write(struct.pack("<III", 0x46546C67, 2, total))
        f.write(struct.pack("<II", len(js), 0x4E4F534A))
        f.write(js)
        f.write(struct.pack("<II", len(bin_chunk), 0x004E4942))
        f.write(bin_chunk)


# --------------------------------------------------------------------- corridor

# Kept in sync with the expected distances in the configuration templates and
# with the table in README.md.
X0, X1 = -10.0, 30.0        # corridor extent along X, open at X0
YW = 2.0                    # walls at Y = +/- YW
ZC = 6.0                    # ceiling height
RAMP_X0, RAMP_X1 = 10.0, 20.0
RAMP_SLOPE = 0.25           # rise per unit X
RAMP_TOP = (RAMP_X1 - RAMP_X0) * RAMP_SLOPE      # 2.5 m
HOLE_X = (4.0, 6.0)
HOLE_Z = (0.5, 1.5)


def build_corridor():
    shell = Mesh()
    # Floor, from X0 to the foot of the ramp, normal +Z
    shell.add_quad((X0, -YW, 0.0), (RAMP_X0, -YW, 0.0),
                   (RAMP_X0, YW, 0.0), (X0, YW, 0.0))
    # Ceiling, normal -Z
    shell.add_quad((X0, -YW, ZC), (X0, YW, ZC), (X1, YW, ZC), (X1, -YW, ZC))
    # Wall at Y = -YW, normal +Y
    shell.add_quad((X0, -YW, 0.0), (X0, -YW, ZC), (X1, -YW, ZC), (X1, -YW, 0.0))
    # Wall at Y = +YW with the window opening, normal -Y
    wall_with_hole(shell, YW, (X0, X1), (0.0, ZC), HOLE_X, HOLE_Z, True)
    # End cap at X1, normal -X
    shell.add_quad((X1, -YW, 0.0), (X1, -YW, ZC), (X1, YW, ZC), (X1, YW, 0.0))

    # Ramp and landing in their own node, authored relative to the ramp foot
    ramp = Mesh()
    dx = RAMP_X1 - RAMP_X0
    ramp.add_quad((0.0, -YW, 0.0), (dx, -YW, RAMP_TOP),
                  (dx, YW, RAMP_TOP), (0.0, YW, 0.0))
    ramp.add_quad((dx, -YW, RAMP_TOP), (X1 - RAMP_X0, -YW, RAMP_TOP),
                  (X1 - RAMP_X0, YW, RAMP_TOP), (dx, YW, RAMP_TOP))
    return [(shell, (0.0, 0.0, 0.0)), (ramp, (RAMP_X0, 0.0, 0.0))]


# ----------------------------------------------------------------- flipped wall

# One wall across the corridor landing, wound the wrong way on purpose: its
# normal points away from a robot arriving from lower X, which is the case
# double_sided="true" has to survive and double_sided="false" cannot.
FLIPPED_WALL_X = 27.0
FLIPPED_WALL_Z = (2.0, 6.0)


def build_flipped_wall():
    m = Mesh()
    z0, z1 = FLIPPED_WALL_Z
    # This winding puts the normal at -X, i.e. facing the incoming robot
    m.add_quad((FLIPPED_WALL_X, -YW, z0), (FLIPPED_WALL_X, -YW, z1),
               (FLIPPED_WALL_X, YW, z1), (FLIPPED_WALL_X, YW, z0))
    # ... and this makes it wrong
    m.flip()
    return [(m, (0.0, 0.0, 0.0))]


# ------------------------------------------------------------------ tiled floor

TILED_X, TILED_Y = 34.0, 10.0
TILED_CELL = 2.0


def build_tiled_floor():
    m = Mesh()
    nu = int(round(TILED_X / TILED_CELL))
    nv = int(round(TILED_Y / TILED_CELL))
    # An odd number of rows puts the X axis half a cell inside a row, so a
    # robot driving along it crosses the diagonal of every cell it enters
    # instead of running along a row boundary.
    m.add_grid((-TILED_X / 2, -TILED_Y / 2, 0.0),
               (TILED_CELL, 0.0, 0.0), (0.0, TILED_CELL, 0.0), nu, nv)
    return [(m, (0.0, 0.0, 0.0))]


# ---------------------------------------------------------------------- terrain

TERRAIN_X, TERRAIN_Y = 130.0, 100.0
TERRAIN_CELL = 0.5
TERRAIN_WALL_Z = 6.0


def build_terrain():
    m = Mesh()
    nu = int(round(TERRAIN_X / TERRAIN_CELL))
    nv = int(round(TERRAIN_Y / TERRAIN_CELL))

    def height(x, y):
        return 0.6 * math.sin(x * 0.08) * math.cos(y * 0.11)

    m.add_grid((-TERRAIN_X / 2, -TERRAIN_Y / 2, 0.0),
               (TERRAIN_X / nu, 0.0, 0.0), (0.0, TERRAIN_Y / nv, 0.0),
               nu, nv, height)
    hx, hy = TERRAIN_X / 2, TERRAIN_Y / 2
    # Perimeter walls, normals pointing inward
    m.add_quad((-hx, -hy, 0.0), (hx, -hy, 0.0),
               (hx, -hy, TERRAIN_WALL_Z), (-hx, -hy, TERRAIN_WALL_Z))
    m.add_quad((hx, hy, 0.0), (-hx, hy, 0.0),
               (-hx, hy, TERRAIN_WALL_Z), (hx, hy, TERRAIN_WALL_Z))
    m.add_quad((-hx, hy, 0.0), (-hx, -hy, 0.0),
               (-hx, -hy, TERRAIN_WALL_Z), (-hx, hy, TERRAIN_WALL_Z))
    m.add_quad((hx, -hy, 0.0), (hx, hy, 0.0),
               (hx, hy, TERRAIN_WALL_Z), (hx, -hy, TERRAIN_WALL_Z))
    # Scattered blocks, so that beams hit something other than the ground and
    # the perimeter. None of them stands on the axes the ray checks use.
    for i in range(10):
        for j in range(10):
            cx = -hx + 6.5 + i * 13.0
            cy = -hy + 5.0 + j * 10.0
            m.add_box((cx - 1.0, cy - 1.0, 0.0), (cx + 1.0, cy + 1.0, 3.0))
    return [(m, (0.0, 0.0, 0.0))]


# ------------------------------------------------------------------------- main


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: make_meshes.py <output directory>")
    outdir = sys.argv[1]
    os.makedirs(outdir, exist_ok=True)
    for name, builder in (("corridor.glb", build_corridor),
                          ("flipped_wall.glb", build_flipped_wall),
                          ("tiled_floor.glb", build_tiled_floor),
                          ("terrain.glb", build_terrain)):
        nodes = builder()
        path = os.path.join(outdir, name)
        write_glb(path, nodes)
        print("%-18s %8d triangles %9d bytes"
              % (name, sum(m.triangles() for m, _ in nodes),
                 os.path.getsize(path)))


if __name__ == "__main__":
    main()
