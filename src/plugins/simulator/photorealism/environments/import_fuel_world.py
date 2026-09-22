#!/usr/bin/env python3
"""Imports a Gazebo Fuel world as a photorealism environment.

A Fuel world is an SDF file that <include>s Fuel models by URI, each
of which is a directory with a model.sdf, COLLADA meshes and textures.
This script downloads the world and every model it references
(recursively: the SubT "... Lights" tiles are themselves wrappers that
include the bare tile and add lamps), composes the SDF pose chain
(world include -> nested include -> link -> visual -> mesh scale ->
COLLADA node, unit and up-axis), and writes, in <out>/<name>/:

  <name>.glb             every <visual>, textured, one glTF node per
                         visual, for the photorealism <scenery><prop>
  <name>.collision.glb   every <collision>, untextured, for the Jolt
                         <mesh> entity
  <name>.lights.xml      every SDF <light> as a photorealism <lights>
                         block, positions and directions in world frame
  <name>.argos           a runnable experiment tying the three together
  <name>.json            bounding box, tile placements, triangle counts

Both glTF files are y-up as the format mandates, so the <prop> takes
orientation="0,0,90" and the <mesh> entity keeps its y_up default.

Materials come from the SDF <material> of each visual: <pbr><metal>
maps (albedo, normal, roughness + metalness combined into one glTF
metallic-roughness texture, emissive), else the Ogre <script> fallback
(albedo only), else the diffuse texture or color of the COLLADA effect.
Textures larger than --texture-size are downscaled.

Usage (any environment with trimesh, pycollada, numpy, pillow):

    python3 -m venv /tmp/fuel-venv
    /tmp/fuel-venv/bin/pip install trimesh pycollada numpy pillow
    /tmp/fuel-venv/bin/python import_fuel_world.py \\
        "OpenRobotics/worlds/Urban Circuit Practice 01" --out .

Fuel downloads are cached in --cache (default ~/.cache/argos3-fuel), so
re-running the conversion with different options is offline.
"""

import argparse
import io
import json
import math
import os
import re
import sys
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
import trimesh
from PIL import Image
from trimesh.visual.material import PBRMaterial

import collada
from collada.common import DaeBrokenRefError, DaeUnsupportedError

FUEL_SERVER = "https://fuel.gazebosim.org/1.0"
# Old worlds still point at the previous host; same API, same content
FUEL_HOSTS = ("fuel.gazebosim.org", "fuel.ignitionrobotics.org")
# SDF lights carry a range but no flux; --lumens is the flux of a lamp
# with this range and other lamps scale with range squared
LIGHT_REFERENCE_RANGE = 20.0

# Rotates the ARGoS frame (z up) onto the glTF frame (y up); the
# inverse is what <prop orientation="0,0,90"> and <mesh y_up="true">
# apply when loading
Z_UP_TO_Y_UP = trimesh.transformations.rotation_matrix(-math.pi / 2.0, [1, 0, 0])


def log(*args):
    print(*args, file=sys.stderr, flush=True)


# --------------------------------------------------------------------
# Fuel access
# --------------------------------------------------------------------

class Fuel:
    """Downloads Fuel models and worlds into a local cache directory."""

    def __init__(self, cache, default_owner):
        self.cache = cache
        self.default_owner = default_owner

    def _fetch(self, kind, owner, name):
        path = os.path.join(self.cache, owner.lower(), kind, name.lower())
        if os.path.isdir(path):
            return path
        url = "{}/{}/{}/{}/tip/{}.zip".format(
            FUEL_SERVER, urllib.parse.quote(owner), kind,
            urllib.parse.quote(name), urllib.parse.quote(name))
        log(f"fetching {kind[:-1]} {owner}/{name}")
        try:
            data = urllib.request.urlopen(url).read()
        except urllib.error.HTTPError as error:
            # model://subt_challenge_cube names the SDF model, while the
            # Fuel resource is titled "SubT Challenge Cube"
            if error.code == 404 and "_" in name:
                return self._fetch(kind, owner, name.replace("_", " "))
            raise
        os.makedirs(path)
        zipfile.ZipFile(io.BytesIO(data)).extractall(path)
        return path

    def model(self, owner, name):
        return self._fetch("models", owner, name)

    def world(self, owner, name):
        return self._fetch("worlds", owner, name)

    def parse_uri(self, uri):
        """Splits a Fuel/model URI into (owner, model name, path inside
        the model or None). Returns None for anything else."""
        parsed = urllib.parse.urlparse(uri)
        if parsed.scheme in ("http", "https") and parsed.netloc in FUEL_HOSTS:
            parts = [urllib.parse.unquote(p) for p in parsed.path.strip("/").split("/")]
            # 1.0/<owner>/models/<name>[/<version>][/files/<path...>]
            if len(parts) >= 4 and parts[2] == "models":
                rest = parts[4:]
                if rest and re.fullmatch(r"\d+|tip", rest[0]):
                    rest = rest[1:]
                if rest and rest[0] == "files":
                    rest = rest[1:]
                return parts[1], parts[3], "/".join(rest) if rest else None
        if parsed.scheme == "model":
            return self.default_owner, urllib.parse.unquote(parsed.netloc), parsed.path.lstrip("/") or None
        return None

    def resolve(self, uri, base_dir):
        """Turns any SDF resource URI into a local file path."""
        uri = uri.strip()
        fuel = self.parse_uri(uri)
        if fuel is not None:
            owner, name, sub = fuel
            # model://<own name>/... inside a model refers to itself
            if uri.startswith("model://") and base_dir and sub and \
               os.path.isfile(os.path.join(base_dir, sub)) and \
               name.lower().replace("_", " ") == os.path.basename(os.path.normpath(base_dir)).lower():
                return os.path.join(base_dir, sub)
            path = self.model(owner, name)
            return os.path.join(path, sub) if sub else path
        if uri.startswith("file://"):
            return uri[len("file://"):]
        if os.path.isabs(uri):
            return uri
        return os.path.normpath(os.path.join(base_dir, uri))


# --------------------------------------------------------------------
# SDF helpers
# --------------------------------------------------------------------

def parse_sdf(path):
    """Some Fuel models start with a blank line before the XML
    declaration (e.g. "Fire Extinguisher"), which ElementTree rejects."""
    with open(path, "rb") as f:
        return ET.fromstring(f.read().lstrip())


def sdf_pose(text):
    """SDF pose 'x y z roll pitch yaw' -> 4x4 matrix (R = Rz Ry Rx)."""
    if text is None or not text.strip():
        return np.eye(4)
    x, y, z, roll, pitch, yaw = [float(v) for v in text.split()]
    m = trimesh.transformations.euler_matrix(roll, pitch, yaw, "sxyz")
    m[:3, 3] = (x, y, z)
    return m


def sdf_vec(text, default):
    if text is None or not text.strip():
        return np.array(default, dtype=float)
    return np.array([float(v) for v in text.split()], dtype=float)


def slugify(name):
    return re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")


# --------------------------------------------------------------------
# COLLADA loading
# --------------------------------------------------------------------

def collada_axis_matrix(up_axis):
    """Maps the COLLADA authoring frame onto the SDF frame (z up)."""
    if up_axis == "Y_UP":
        return np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], dtype=float)
    if up_axis == "X_UP":
        return np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]], dtype=float)
    return np.eye(4)


class DaePrimitive:
    """One triangle batch of one COLLADA node, unwelded."""
    __slots__ = ("vertices", "normals", "uv", "diffuse_texture", "diffuse_color")

    def __init__(self, vertices, normals, uv, diffuse_texture, diffuse_color):
        self.vertices = vertices
        self.normals = normals
        self.uv = uv
        self.diffuse_texture = diffuse_texture
        self.diffuse_color = diffuse_color


class DaeSubmesh:
    __slots__ = ("names", "primitives")

    def __init__(self, names, primitives):
        self.names = names
        self.primitives = primitives


class DaeFile:
    """A COLLADA file flattened into submeshes in the SDF model frame
    (meters, z up), with the node transforms applied."""

    def __init__(self, path):
        self.path = path
        dae = collada.Collada(path, ignore=[DaeUnsupportedError, DaeBrokenRefError])
        frame = collada_axis_matrix(dae.assetInfo.upaxis)
        frame[:3, :3] *= float(dae.assetInfo.unitmeter or 1.0)
        self.submeshes = []
        if dae.scene is not None:
            for node in dae.scene.nodes:
                self._walk(node, frame)
        else:
            for geometry in dae.geometries:
                self._add(geometry, [], frame, {})

    def _walk(self, node, parent):
        if isinstance(node, collada.scene.Node):
            world = parent @ node.matrix
            names = {node.id, node.xmlnode.get("name")}
            for child in node.children:
                if isinstance(child, collada.scene.GeometryNode):
                    self._add(child.geometry, names, world,
                              {m.symbol: m.target for m in child.materials})
                elif isinstance(child, collada.scene.NodeNode):
                    self._walk(child.node, world)
                elif isinstance(child, collada.scene.Node):
                    self._walk(child, world)
        elif isinstance(node, collada.scene.GeometryNode):
            self._add(node.geometry, set(), parent,
                      {m.symbol: m.target for m in node.materials})

    def _add(self, geometry, names, world, materials):
        primitives = []
        for primitive in geometry.primitives:
            if isinstance(primitive, collada.triangleset.TriangleSet):
                tris = primitive
            elif isinstance(primitive, (collada.polylist.Polylist, collada.polygons.Polygons)):
                tris = primitive.triangleset()
            else:
                continue
            if len(tris) == 0:
                continue
            vertices = tris.vertex[tris.vertex_index].reshape(-1, 3)
            vertices = trimesh.transform_points(vertices, world)
            normals = None
            if tris.normal is not None and tris.normal_index is not None:
                normals = tris.normal[tris.normal_index].reshape(-1, 3)
                normals = normals @ np.linalg.inv(world[:3, :3])
                lengths = np.linalg.norm(normals, axis=1)
                normals[lengths > 0] /= lengths[lengths > 0, None]
            uv = None
            if tris.texcoordset:
                uv = tris.texcoordset[0][tris.texcoord_indexset[0]].reshape(-1, 2)
            texture, color = None, None
            material = materials.get(tris.material)
            if material is not None and material.effect is not None:
                diffuse = material.effect.diffuse
                if isinstance(diffuse, collada.material.Map):
                    image_path = diffuse.sampler.surface.image.path
                    texture = urllib.parse.unquote(image_path)
                elif diffuse is not None:
                    color = tuple(float(c) for c in diffuse[:3])
            primitives.append(DaePrimitive(vertices, normals, uv, texture, color))
        if primitives:
            names = {n for n in names if n} | {geometry.id, geometry.name}
            self.submeshes.append(DaeSubmesh({n for n in names if n}, primitives))

    def select(self, submesh_name):
        if submesh_name is None:
            return self.submeshes
        found = [s for s in self.submeshes if submesh_name in s.names]
        if not found:
            raise ValueError("no submesh \"{}\" in {} (have: {})".format(
                submesh_name, self.path,
                ", ".join(sorted({n for s in self.submeshes for n in s.names}))))
        return found


# --------------------------------------------------------------------
# Materials
# --------------------------------------------------------------------

class Textures:
    """Loads, downscales and caches texture images and PBR materials."""

    def __init__(self, fuel, max_size):
        self.fuel = fuel
        self.max_size = max_size
        self.images = {}
        self.materials = {}
        self.missing = set()

    def image(self, path, mode):
        key = (path, mode)
        if key in self.images:
            return self.images[key]
        image = None
        if os.path.isfile(path):
            image = Image.open(path)
            image.load()
            if self.max_size and max(image.size) > self.max_size:
                scale = self.max_size / max(image.size)
                image = image.resize((max(1, round(image.width * scale)),
                                      max(1, round(image.height * scale))),
                                     Image.LANCZOS)
            image = image.convert(mode)
        elif path not in self.missing:
            self.missing.add(path)
            log(f"warning: texture not found: {path}")
        self.images[key] = image
        return image

    @staticmethod
    def as_jpeg(image, quality=90):
        """trimesh re-encodes anything that is not already a JPEG as
        PNG; a JPEG round trip keeps the file size sane."""
        if image is None:
            return None
        buffer = io.BytesIO()
        image.save(buffer, format="JPEG", quality=quality)
        buffer.seek(0)
        return Image.open(buffer)

    TEXTURE_KEYS = ("albedo", "normal", "roughness", "metalness", "emissive")

    def material(self, spec):
        """spec: dict with optional keys albedo, normal, roughness,
        metalness, emissive (URIs), base_dir, diffuse (rgb), emissive_color
        (rgb), name. Materials are shared between every visual that
        resolves to the same textures and factors, whatever model
        directory they were named from."""
        base_dir = spec.get("base_dir", "")
        paths = {k: self.fuel.resolve(spec[k], base_dir) for k in self.TEXTURE_KEYS if spec.get(k)}
        diffuse = tuple(spec.get("diffuse", (1.0, 1.0, 1.0)))
        emissive_color = tuple(spec.get("emissive_color", (0.0, 0.0, 0.0)))
        key = json.dumps({"paths": paths, "diffuse": diffuse, "emissive": emissive_color}, sort_keys=True)
        if key in self.materials:
            return self.materials[key]
        material = PBRMaterial(name=spec.get("name", "material"))
        material.baseColorFactor = [diffuse[0], diffuse[1], diffuse[2], 1.0]
        material.metallicFactor = 0.0
        material.roughnessFactor = 1.0
        albedo = self.image(paths["albedo"], "RGB") if "albedo" in paths else None
        if albedo is not None:
            material.baseColorTexture = self.as_jpeg(albedo)
        normal = self.image(paths["normal"], "RGB") if "normal" in paths else None
        if normal is not None:
            material.normalTexture = self.as_jpeg(normal)
        roughness = self.image(paths["roughness"], "L") if "roughness" in paths else None
        metalness = self.image(paths["metalness"], "L") if "metalness" in paths else None
        if roughness is not None or metalness is not None:
            size = (roughness or metalness).size
            if roughness is None:
                roughness = Image.new("L", size, 255)
            if metalness is None:
                metalness = Image.new("L", size, 0)
            elif metalness.size != roughness.size:
                metalness = metalness.resize(roughness.size, Image.LANCZOS)
            # glTF packs roughness in G and metalness in B
            packed = Image.merge("RGB", (Image.new("L", roughness.size, 0), roughness, metalness))
            material.metallicRoughnessTexture = self.as_jpeg(packed)
            material.metallicFactor = 1.0
        emissive = self.image(paths["emissive"], "RGB") if "emissive" in paths else None
        if emissive is not None:
            material.emissiveTexture = self.as_jpeg(emissive)
            material.emissiveFactor = [1.0, 1.0, 1.0]
        elif any(c > 0 for c in emissive_color):
            material.emissiveFactor = list(emissive_color[:3])
        self.materials[key] = material
        return material


def parse_ogre_script(script_dirs, material_name):
    """Finds 'material <name> { ... texture <file> }' in Ogre .material
    scripts and returns the texture file name, or None."""
    for directory in script_dirs:
        if not os.path.isdir(directory):
            continue
        for entry in sorted(os.listdir(directory)):
            if not entry.endswith(".material"):
                continue
            with open(os.path.join(directory, entry), errors="replace") as f:
                text = f.read()
            match = re.search(r"material\s+" + re.escape(material_name) + r"\s*\{", text)
            if match is None:
                continue
            depth, i = 0, match.end() - 1
            while i < len(text):
                if text[i] == "{":
                    depth += 1
                elif text[i] == "}":
                    depth -= 1
                    if depth == 0:
                        break
                i += 1
            body = text[match.end():i]
            texture = re.search(r"\btexture\s+(\S+)", body)
            return texture.group(1) if texture else None
    return None


def material_spec(visual, model_dir, fuel):
    """Reads the SDF <material> of a visual into a spec for Textures.
    Returns None when the visual has no usable material and the COLLADA
    effect should be used instead."""
    material = visual.find("material")
    if material is None:
        return None
    spec = {"base_dir": model_dir}
    diffuse = material.findtext("diffuse")
    if diffuse:
        spec["diffuse"] = tuple(sdf_vec(diffuse, (1, 1, 1))[:3].tolist())
    emissive = material.findtext("emissive")
    if emissive:
        spec["emissive_color"] = tuple(sdf_vec(emissive, (0, 0, 0))[:3].tolist())
    pbr = material.find("pbr")
    if pbr is not None:
        workflow = pbr.find("metal")
        if workflow is None:
            workflow = pbr.find("specular")
        if workflow is not None:
            for key, tag in (("albedo", "albedo_map"), ("normal", "normal_map"),
                             ("roughness", "roughness_map"), ("metalness", "metalness_map"),
                             ("emissive", "emissive_map")):
                uri = workflow.findtext(tag)
                if uri:
                    spec[key] = uri.strip()
            if "albedo" in spec:
                return spec
    script = material.find("script")
    if script is not None and script.findtext("name"):
        dirs = [fuel.resolve(u.text, model_dir) for u in script.findall("uri") if u.text]
        texture = parse_ogre_script(dirs, script.findtext("name").strip())
        if texture:
            for directory in dirs:
                candidate = os.path.join(directory, texture)
                if os.path.isfile(candidate):
                    spec["albedo"] = candidate
                    return spec
            for directory in dirs:
                candidate = os.path.join(os.path.dirname(directory), "textures", texture)
                if os.path.isfile(candidate):
                    spec["albedo"] = candidate
                    return spec
    if "diffuse" in spec or "emissive_color" in spec:
        return spec
    return None


# --------------------------------------------------------------------
# World traversal
# --------------------------------------------------------------------

class Importer:

    def __init__(self, fuel, textures, double_sided):
        self.fuel = fuel
        self.textures = textures
        self.double_sided = double_sided
        self.dae_cache = {}
        self.visuals = []      # (name, Trimesh)
        self.collisions = []   # (name, Trimesh)
        self.lights = []       # dicts
        self.placements = []   # top-level includes
        self.skipped = []

    def dae(self, path):
        if path not in self.dae_cache:
            self.dae_cache[path] = DaeFile(path)
        return self.dae_cache[path]

    # -- geometry -----------------------------------------------------

    def primitive_mesh(self, geometry, model_dir, world):
        """<geometry> with a box/cylinder/sphere child -> Trimesh in
        the world frame, or None."""
        box = geometry.find("box")
        if box is not None:
            size = sdf_vec(box.findtext("size"), (1, 1, 1))
            mesh = trimesh.creation.box(extents=size)
        elif geometry.find("cylinder") is not None:
            cylinder = geometry.find("cylinder")
            mesh = trimesh.creation.cylinder(radius=float(cylinder.findtext("radius")),
                                             height=float(cylinder.findtext("length")))
        elif geometry.find("sphere") is not None:
            mesh = trimesh.creation.icosphere(radius=float(geometry.find("sphere").findtext("radius")))
        else:
            return None
        mesh.apply_transform(world)
        return mesh

    def mesh_geometry(self, geometry, model_dir, world, with_material, material):
        """<geometry><mesh> -> list of Trimesh in the world frame."""
        mesh_node = geometry.find("mesh")
        uri = mesh_node.findtext("uri")
        path = self.fuel.resolve(uri, model_dir)
        if not os.path.isfile(path):
            raise FileNotFoundError(path)
        scale = np.diag(list(sdf_vec(mesh_node.findtext("scale"), (1, 1, 1))) + [1.0])
        world = world @ scale
        submesh = mesh_node.find("submesh")
        submesh_name = submesh.findtext("name").strip() if submesh is not None else None
        if submesh is not None and (submesh.findtext("center") or "false").strip().lower() == "true":
            log(f"warning: <submesh><center> not supported ({path}:{submesh_name}); ignored")
        dae = self.dae(path)
        meshes = []
        dae_dir = os.path.dirname(path)
        for sub in dae.select(submesh_name):
            for primitive in sub.primitives:
                vertices = trimesh.transform_points(primitive.vertices, world)
                faces = np.arange(len(vertices)).reshape(-1, 3)
                mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
                if primitive.normals is not None:
                    normals = primitive.normals @ np.linalg.inv(world[:3, :3])
                    lengths = np.linalg.norm(normals, axis=1)
                    normals[lengths > 0] /= lengths[lengths > 0, None]
                    mesh.vertex_normals = normals
                if with_material:
                    spec = material
                    if spec is None:
                        spec = {"base_dir": dae_dir}
                        if primitive.diffuse_texture:
                            spec["albedo"] = primitive.diffuse_texture
                        if primitive.diffuse_color:
                            spec["diffuse"] = primitive.diffuse_color
                    pbr = self.textures.material(spec)
                    uv = primitive.uv
                    if uv is not None:
                        # COLLADA v runs bottom-up like glTF expects after
                        # trimesh's export flip, so pass it through
                        mesh.visual = trimesh.visual.TextureVisuals(uv=uv, material=pbr)
                    else:
                        mesh.visual = trimesh.visual.TextureVisuals(material=pbr)
                mesh.merge_vertices()
                meshes.append(mesh)
        return meshes

    def geometry_meshes(self, element, model_dir, world, with_material, material=None):
        geometry = element.find("geometry")
        if geometry is None:
            return []
        if geometry.find("mesh") is not None:
            return self.mesh_geometry(geometry, model_dir, world, with_material, material)
        mesh = self.primitive_mesh(geometry, model_dir, world)
        if mesh is None:
            kinds = [c.tag for c in geometry]
            self.skipped.append("{} geometry {}".format(element.get("name"), kinds))
            return []
        if with_material:
            spec = material or {"base_dir": model_dir, "diffuse": (0.7, 0.7, 0.7)}
            mesh.visual = trimesh.visual.TextureVisuals(material=self.textures.material(spec))
        return [mesh]

    # -- SDF tree -----------------------------------------------------

    def add_light(self, light, world, prefix):
        pose = world @ sdf_pose(light.findtext("pose"))
        direction = pose[:3, :3] @ sdf_vec(light.findtext("direction"), (0, 0, -1))
        direction /= max(np.linalg.norm(direction), 1e-9)
        attenuation = light.find("attenuation")
        spot = light.find("spot")
        entry = {
            "name": "{}/{}".format(prefix, light.get("name")),
            "type": light.get("type", "point"),
            "position": pose[:3, 3].tolist(),
            "direction": direction.tolist(),
            "color": sdf_vec(light.findtext("diffuse"), (1, 1, 1, 1))[:3].tolist(),
            "range": float(attenuation.findtext("range")) if attenuation is not None and attenuation.findtext("range") else 10.0,
            "cast_shadows": (light.findtext("cast_shadows") or "false").strip().lower() == "true",
        }
        if spot is not None:
            # SDF cone angles are full angles; the medium takes half-angles
            entry["inner_angle"] = math.degrees(float(spot.findtext("inner_angle") or 0.5)) / 2.0
            entry["outer_angle"] = math.degrees(float(spot.findtext("outer_angle") or 1.0)) / 2.0
        self.lights.append(entry)

    def add_model(self, model, model_dir, world, prefix):
        """Walks a <model> element: links, nested models, includes."""
        for link in model.findall("link"):
            link_world = world @ sdf_pose(link.findtext("pose"))
            link_name = "{}/{}".format(prefix, link.get("name"))
            for visual in link.findall("visual"):
                visual_world = link_world @ sdf_pose(visual.findtext("pose"))
                spec = material_spec(visual, model_dir, self.fuel)
                if spec is not None:
                    spec["name"] = "{}/{}".format(link_name, visual.get("name"))
                for i, mesh in enumerate(self.geometry_meshes(visual, model_dir, visual_world, True, spec)):
                    name = "{}/{}".format(link_name, visual.get("name"))
                    self.visuals.append((name if i == 0 else f"{name}#{i}", mesh))
            for collision in link.findall("collision"):
                collision_world = link_world @ sdf_pose(collision.findtext("pose"))
                for i, mesh in enumerate(self.geometry_meshes(collision, model_dir, collision_world, False)):
                    name = "{}/{}".format(link_name, collision.get("name"))
                    self.collisions.append((name if i == 0 else f"{name}#{i}", mesh))
            for light in link.findall("light"):
                self.add_light(light, link_world, link_name)
        for light in model.findall("light"):
            self.add_light(light, world, prefix)
        for nested in model.findall("model"):
            self.add_model(nested, model_dir, world @ sdf_pose(nested.findtext("pose")),
                           "{}/{}".format(prefix, nested.get("name")))
        for include in model.findall("include"):
            self.add_include(include, world, prefix)

    def add_include(self, include, world, prefix, top_level=False):
        uri = include.findtext("uri").strip()
        fuel = self.fuel.parse_uri(uri)
        if fuel is None:
            model_dir = self.fuel.resolve(uri, "")
        else:
            model_dir = self.fuel.model(fuel[0], fuel[1])
        sdf_path = os.path.join(model_dir, "model.sdf")
        root = parse_sdf(sdf_path)
        model = root.find("model")
        if model is None:
            self.skipped.append(f"{uri}: no <model> in model.sdf")
            return
        name = (include.findtext("name") or model.get("name") or os.path.basename(model_dir)).strip()
        pose = sdf_pose(include.findtext("pose"))
        model_world = world @ pose
        full_name = "{}/{}".format(prefix, name) if prefix else name
        before = (len(self.visuals), len(self.collisions), len(self.lights))
        self.add_model(model, model_dir, model_world, full_name)
        if top_level:
            self.placements.append({
                "name": name,
                "model": os.path.basename(model_dir),
                "uri": uri,
                "pose": (include.findtext("pose") or "0 0 0 0 0 0").split(),
                "visuals": len(self.visuals) - before[0],
                "collisions": len(self.collisions) - before[1],
                "lights": len(self.lights) - before[2],
            })

    def add_world(self, world_sdf):
        root = parse_sdf(world_sdf)
        world = root.find("world")
        if world is None:
            raise ValueError(f"{world_sdf} has no <world>")
        for include in world.findall("include"):
            self.add_include(include, np.eye(4), "", top_level=True)
        for model in world.findall("model"):
            self.add_model(model, os.path.dirname(world_sdf),
                           sdf_pose(model.findtext("pose")), model.get("name"))
        for light in world.findall("light"):
            self.add_light(light, np.eye(4), "world")
        return world


# --------------------------------------------------------------------
# Output
# --------------------------------------------------------------------

def export_scene(meshes, path, double_sided):
    scene = trimesh.Scene()
    for name, mesh in meshes:
        if double_sided and mesh.visual.kind == "texture":
            mesh.visual.material.doubleSided = True
        scene.add_geometry(mesh, node_name=name, geom_name=name)
    scene.apply_transform(Z_UP_TO_Y_UP)
    scene.export(path)
    return os.path.getsize(path)


def light_lumens(light, lumens):
    """SDF gives lights a reach, not a flux. Scaling the flux with the
    square of the range gives every lamp the same illuminance at its
    designed reach: 'lumens' is the flux of a 20 m lamp."""
    return lumens * (light["range"] / LIGHT_REFERENCE_RANGE) ** 2


def floor_below(point, meshes):
    """Highest z of the collision geometry straight below point, or
    None. Moller-Trumbore over the triangles whose xy box contains the
    point; no spatial index needed for a single query."""
    best = None
    for _, mesh in meshes:
        lo, hi = mesh.bounds
        if not (lo[0] <= point[0] <= hi[0] and lo[1] <= point[1] <= hi[1] and lo[2] < point[2]):
            continue
        tri = mesh.triangles
        inside = (tri[:, :, 0].min(axis=1) <= point[0]) & (tri[:, :, 0].max(axis=1) >= point[0]) & \
                 (tri[:, :, 1].min(axis=1) <= point[1]) & (tri[:, :, 1].max(axis=1) >= point[1])
        tri = tri[inside]
        if len(tri) == 0:
            continue
        e1 = tri[:, 1] - tri[:, 0]
        e2 = tri[:, 2] - tri[:, 0]
        direction = np.array([0.0, 0.0, -1.0])
        p = np.cross(direction, e2)
        det = np.einsum("ij,ij->i", e1, p)
        ok = np.abs(det) > 1e-12
        inv = np.zeros_like(det)
        inv[ok] = 1.0 / det[ok]
        s = point - tri[:, 0]
        u = np.einsum("ij,ij->i", s, p) * inv
        q = np.cross(s, e1)
        v = np.einsum("j,ij->i", direction, q) * inv
        t = np.einsum("ij,ij->i", e2, q) * inv
        hit = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 0)
        if hit.any():
            z = point[2] - t[hit].min()
            best = z if best is None else max(best, z)
    return best


def bounds(meshes):
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)
    for _, mesh in meshes:
        lo = np.minimum(lo, mesh.bounds[0])
        hi = np.maximum(hi, mesh.bounds[1])
    return lo, hi


def fmt(values, digits=3):
    return ",".join("{:.{}f}".format(v, digits).rstrip("0").rstrip(".") if isinstance(v, float) else str(v) for v in values)


def lights_xml(lights, lumens, indent="        "):
    lines = []
    for light in lights:
        attrs = ['position="{}"'.format(fmt(light["position"]))]
        if light["type"] == "spot":
            attrs.append('direction="{}"'.format(fmt(light["direction"])))
        attrs.append('intensity="{:.0f}"'.format(light_lumens(light, lumens)))
        attrs.append('falloff="{}"'.format(fmt([light["range"]], 1)))
        attrs.append('color="{}"'.format(fmt(light["color"])))
        if light["type"] == "spot":
            attrs.append('inner_angle="{}"'.format(fmt([light["inner_angle"]], 1)))
            attrs.append('outer_angle="{}"'.format(fmt([min(light["outer_angle"], 89.0)], 1)))
        if light["cast_shadows"]:
            attrs.append('cast_shadows="true"')
        tag = "spot" if light["type"] == "spot" else "point"
        lines.append('{}<!-- {} -->'.format(indent, light["name"]))
        lines.append('{}<{} {} />'.format(indent, tag, " ".join(attrs)))
    return "\n".join(lines)


ARGOS_TEMPLATE = """<?xml version="1.0" ?>
<argos-configuration>

  <!-- GENERATED by import_fuel_world.py from the Gazebo Fuel world
       "{title}" ({owner}); regenerate rather than edit.

       A stand-alone tour of the environment: no robots, only the
       scenery, its lights and the collision mesh. Run it from this
       directory:

           argos3 -c {name}.argos            interactive viewer
           argos3 -z -c {name}.argos         headless, dumps frames/

       Drop robots into the <arena> and give them photorealistic_camera
       or photorealistic_lidar sensors on medium "pr" to use it in an
       experiment. The camera starts under the first lamp; the world's
       first placement, "{start_name}", is at {start} (SDF frame) and
       every placement is listed in {name}.json. -->

  <framework>
    <system threads="0" />
    <experiment length="{length}" ticks_per_second="10" random_seed="1" />
  </framework>

  <controllers />

  <arena size="{arena_size}" center="{arena_center}">
    <!-- Physics: every SDF <collision> of the world, as one static
         triangle mesh. y_up is the default: the file is glTF y-up. -->
    <mesh id="world" file="{name}.collision.glb" />
  </arena>

  <physics_engines>
    <jolt id="jolt">
      <gravity g="9.81" />
    </jolt>
  </physics_engines>

  <media>
    <!-- draw_floor="false": the ground is the tiles' own floors, not
         the ARGoS plane (which would cut through the lower levels). -->
    <photorealism id="pr" backend="vulkan" draw_floor="false">
      <!-- Underground: no sun, black sky. The SDF scene has
           ambient 0.1 and background black; here the tiles are lit by
           their own lamps only. -->
      <sun direction="0,0,-1" intensity="0" cast_shadows="false" />
      <skybox color="0,0,0" />
      <!-- Lamp-lit interior: opened up {stops} stops from sunny 16. -->
      <exposure aperture="{aperture}" shutter_speed="{shutter}" sensitivity="{iso}" />
      <lights>
        <!-- Most of these worlds are dark away from the few lit tiles,
             as in the competition, where robots carried their own
             lamps. A light with entity="<robot id>" is a headlight
             mounted on that robot (position and direction in its body
             frame, +x forward), e.g. for a spot or a bunker:
             <spot entity="sp0" position="0.45,0,0.2" direction="1,0,-0.15"
                   intensity="3000" falloff="25" inner_angle="20"
                   outer_angle="35" color="1.0,0.95,0.85" /> -->
        <!-- {nlights} SDF lights, world frame; {lumens:.0f} lm for a 20 m
             range, scaled with range squared -->
{lights}
      </lights>
      <scenery>
        <!-- Every SDF <visual>, y-up glTF hence orientation 0,0,90 -->
        <prop model="{name}.glb" position="0,0,0" orientation="0,0,90" scale="1" />
      </scenery>
      <debug_camera position="{cam_pos}" look_at="{cam_look}" fov="70"
                    resolution="960,540" period="10" dump="frames" />
    </photorealism>
  </media>

  <visualization>
    <!-- The flashlight (F toggles) rides on the free-fly camera and is
         seen by this window only, never by robot sensors: most of the
         map is unlit. -->
    <filament medium="pr" resolution="1280,720" speed="1"
              near="0.1" far="300" flashlight="true"
              position="{cam_pos}" look_at="{cam_look}" />
  </visualization>

</argos-configuration>
"""


def exposure_for(stops):
    """Sunny 16 (f/16, 1/100 s, ISO 100) opened up by 'stops' stops,
    split between aperture, shutter and ISO like a photographer would."""
    aperture = 16.0 / (2 ** (min(stops, 6) / 2.0))
    remaining = max(0.0, stops - 6)
    shutter = 0.01 * (2 ** min(remaining, 2))
    iso = 100 * (2 ** max(0.0, remaining - 2))
    return round(aperture, 1), round(shutter, 4), int(round(iso))


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                     formatter_class=argparse.RawDescriptionHelpFormatter,
                                     epilog=__doc__)
    parser.add_argument("world", help='"<owner>/worlds/<name>", a Fuel URL, or a local .sdf file')
    parser.add_argument("--out", default=".", help="output root; files go to <out>/<name>/")
    parser.add_argument("--name", help="environment name (default: slug of the world name)")
    parser.add_argument("--cache", default=os.path.expanduser("~/.cache/argos3-fuel"))
    parser.add_argument("--owner", default="OpenRobotics", help="owner for model:// URIs")
    parser.add_argument("--texture-size", type=int, default=1024,
                        help="downscale textures to at most this many pixels per side (0 = keep)")
    parser.add_argument("--lumens", type=float, default=40000.0,
                        help="luminous flux of an SDF light with a 20 m range; other ranges "
                             "scale with range squared (SDF has no photometric units; the "
                             "default renders the SubT urban tiles at a mean luminance of "
                             "about 0.24 with the default --stops)")
    parser.add_argument("--stops", type=float, default=10.0,
                        help="exposure: stops opened up from sunny 16 in the generated .argos")
    parser.add_argument("--double-sided", action="store_true",
                        help="mark every glTF material double sided (assets with holes seen from outside)")
    args = parser.parse_args()

    fuel = Fuel(args.cache, args.owner)

    # Locate the world SDF
    title, owner = None, args.owner
    if os.path.isfile(args.world):
        world_sdf = args.world
        title = os.path.splitext(os.path.basename(world_sdf))[0]
    else:
        match = re.fullmatch(r"(?:https?://[^/]+/1\.0/)?([^/]+)/worlds/([^/]+)(?:/.*)?", args.world.strip())
        if match is None:
            parser.error("world must be a local .sdf, '<owner>/worlds/<name>' or a Fuel URL")
        owner, title = urllib.parse.unquote(match.group(1)), urllib.parse.unquote(match.group(2))
        fuel.default_owner = owner
        world_dir = fuel.world(owner, title)
        sdfs = [f for f in os.listdir(world_dir) if f.endswith(".sdf")]
        if len(sdfs) != 1:
            parser.error(f"expected one .sdf in {world_dir}, found {sdfs}")
        world_sdf = os.path.join(world_dir, sdfs[0])
    name = args.name or slugify(title)
    out_dir = os.path.join(args.out, name)
    os.makedirs(out_dir, exist_ok=True)

    textures = Textures(fuel, args.texture_size)
    importer = Importer(fuel, textures, args.double_sided)
    importer.add_world(world_sdf)
    if not importer.visuals:
        parser.error("the world produced no visual geometry")

    lo, hi = bounds(importer.visuals)
    visual_tris = sum(len(m.faces) for _, m in importer.visuals)
    collision_tris = sum(len(m.faces) for _, m in importer.collisions)
    log(f"{len(importer.placements)} placements, {len(importer.visuals)} visuals "
        f"({visual_tris} triangles), {len(importer.collisions)} collisions "
        f"({collision_tris} triangles), {len(importer.lights)} lights, "
        f"{len(textures.materials)} materials")
    log("bounds x [{:.1f}, {:.1f}] y [{:.1f}, {:.1f}] z [{:.1f}, {:.1f}]".format(
        lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))
    for entry in importer.skipped:
        log("skipped:", entry)

    visual_path = os.path.join(out_dir, f"{name}.glb")
    size = export_scene(importer.visuals, visual_path, args.double_sided)
    log(f"{visual_path}: {size / 1048576:.1f} MiB")
    collision_path = os.path.join(out_dir, f"{name}.collision.glb")
    if importer.collisions:
        size = export_scene(importer.collisions, collision_path, False)
        log(f"{collision_path}: {size / 1048576:.1f} MiB")

    lights = lights_xml(importer.lights, args.lumens)
    with open(os.path.join(out_dir, f"{name}.lights.xml"), "w") as f:
        f.write("<lights>\n{}\n</lights>\n".format(lights_xml(importer.lights, args.lumens, "  ")))

    # A start pose: the first placement is the staging area by SubT
    # convention; otherwise the middle of the world
    start = [float(v) for v in importer.placements[0]["pose"][:3]] if importer.placements else ((lo + hi) / 2).tolist()
    margin = 2.0
    arena_lo, arena_hi = lo - margin, hi + margin
    aperture, shutter, iso = exposure_for(args.stops)
    # Viewpoint: under the first lamp at eye height above the floor the
    # collision mesh has there, looking at the next lamp of the same room
    # when there is one; a world without lamps is viewed from the start
    # pose instead
    if importer.lights:
        first = np.array(importer.lights[0]["position"])
        floor = floor_below(first, importer.collisions)
        eye = min(first[2] - 0.5, floor + 1.6) if floor is not None else first[2] - 3.2
        cam_pos = [first[0], first[1], eye]
        others = [np.array(l["position"]) for l in importer.lights[1:]]
        near = [p for p in others if 3.0 < np.linalg.norm(p[:2] - first[:2]) < 30.0 and abs(p[2] - first[2]) < 2.0]
        target = near[0] if near else first + np.array([5.0, 0.0, 0.0])
        cam_look = [target[0], target[1], eye - 0.4]
    else:
        cam_pos = [start[0] + 1.0, start[1], start[2] + 1.6]
        cam_look = [start[0] + 6.0, start[1], start[2] + 1.0]
    with open(os.path.join(out_dir, f"{name}.argos"), "w") as f:
        f.write(ARGOS_TEMPLATE.format(
            title=title, owner=owner, name=name, start=fmt(start, 1), length=0,
            start_name=importer.placements[0]["name"] if importer.placements else "world",
            arena_size=fmt((arena_hi - arena_lo).tolist(), 1),
            arena_center=fmt(((arena_lo + arena_hi) / 2).tolist(), 1),
            stops=fmt([args.stops], 1), aperture=aperture, shutter=shutter, iso=iso,
            nlights=len(importer.lights), lumens=args.lumens, lights=lights,
            cam_pos=fmt(cam_pos, 2), cam_look=fmt(cam_look, 2)))

    with open(os.path.join(out_dir, f"{name}.json"), "w") as f:
        json.dump({
            "world": title, "owner": owner, "source": world_sdf,
            "bounds_min": lo.tolist(), "bounds_max": hi.tolist(),
            "visual_triangles": int(visual_tris), "collision_triangles": int(collision_tris),
            "materials": len(textures.materials), "texture_size": args.texture_size,
            "placements": importer.placements, "lights": importer.lights,
            "skipped": importer.skipped,
        }, f, indent=1)
    log(f"wrote {out_dir}/{name}.{{glb,collision.glb,lights.xml,argos,json}}")


if __name__ == "__main__":
    main()
