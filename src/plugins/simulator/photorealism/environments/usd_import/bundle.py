"""Assemble reusable scene geometry and export ARGoS environment bundles."""
from dataclasses import dataclass
from pathlib import Path
import hashlib
import json
import struct
import numpy as np
import trimesh
from pxr import UsdLux, UsdPhysics
from .scene import iter_occurrences, is_visual, is_collider, warn
from .geometry import convert_geometry
from .materials import convert_material, attach_material
from .lights import Lighting, convert_lights


@dataclass
class Bundle:
    visual: trimesh.Scene
    collision: trimesh.Scene | None
    lighting: Lighting
    bounds: np.ndarray
    report: dict


MATERIAL_FIELDS=('baseColorFactor','metallicFactor','roughnessFactor','emissiveFactor',
                 'alphaMode','alphaCutoff','doubleSided','baseColorTexture','metallicRoughnessTexture',
                 'emissiveTexture','normalTexture','occlusionTexture')


def _material_digest(mesh):
    digest=hashlib.sha256()
    material=mesh.visual.material
    for name in MATERIAL_FIELDS:
        value=getattr(material,name,None)
        digest.update(name.encode())
        if value is None: digest.update(b'null')
        else:
            a=np.asarray(value)
            digest.update(str((a.shape,a.dtype)).encode()); digest.update(a.tobytes())
    digest.update(json.dumps(mesh.metadata.get('usd_gltf_overrides',{}),sort_keys=True).encode())
    return digest.hexdigest()


def _geometry_digest(mesh):
    digest=hashlib.sha256()
    for a in (mesh.vertices,mesh.faces,mesh.vertex_normals):
        digest.update(str(a.shape).encode()); digest.update(np.ascontiguousarray(a).tobytes())
    if mesh.visual.kind=='texture':
        digest.update(_material_digest(mesh).encode())
        if mesh.visual.uv is not None: digest.update(np.asarray(mesh.visual.uv).tobytes())
    elif mesh.visual.kind=='vertex': digest.update(mesh.visual.vertex_colors.tobytes())
    return digest.hexdigest()


def _add(scene,mesh,transform,key):
    transform=transform.copy()
    if np.linalg.det(transform[:3,:3])<0:
        reflection=np.diag([-1.,1.,1.,1.])
        mesh=mesh.copy(include_cache=True); mesh.apply_transform(reflection)
        transform=transform @ reflection
    if mesh.visual.kind=='texture': mesh.visual.material.name='usd-material-'+_material_digest(mesh)
    geometry='usd-geometry-'+_geometry_digest(mesh)
    if geometry not in scene.geometry: scene.geometry[geometry]=mesh
    scene.graph.update(frame_to=key,frame_from=scene.graph.base_frame,matrix=transform,geometry=geometry)


def build_bundle(ctx):
    occurrences=list(iter_occurrences(ctx))
    authored=[o for o in occurrences if is_collider(o,ctx)]
    mode=ctx.options.collision
    if mode=='authored' and not authored: raise ValueError('authored collision mode: no enabled colliders')
    if mode=='auto':
        mode='authored' if authored else 'visual'
        if not authored: warn(ctx,'collision_fallback','/','no authored colliders; visual geometry used as static collision')
    ctx.report['collision_policy']='visual-fallback' if ctx.options.collision=='auto' and not authored else mode
    visual=trimesh.Scene(); collision=trimesh.Scene() if mode!='none' else None
    for occ in occurrences:
        if UsdLux.LightAPI(occ.prim): continue
        visible=is_visual(occ,ctx)
        collide=(mode=='authored' and is_collider(occ,ctx)) or (mode=='visual' and visible)
        if not visible and not collide: continue
        if occ.prim.HasAPI(UsdPhysics.RigidBodyAPI):
            warn(ctx,'static_body',occ.key,'rigid-body dynamics ignored; geometry imported statically')
        parts=convert_geometry(occ,ctx)
        if collide and not parts: raise ValueError(f'{occ.key}: selected collider has no valid triangles')
        for part in parts:
            # Include full USD identity rather than lossy sanitized labels in node names.
            key=occ.key+'|'+part.key
            if visible:
                mesh=attach_material(part,convert_material(part.binding_prim,ctx),ctx)
                _add(visual,mesh,occ.transform,key)
            if collide:
                mesh=part.mesh.copy(include_cache=True); mesh.visual=trimesh.visual.ColorVisuals(mesh)
                _add(collision,mesh,occ.transform,key)
            ctx.report['sources'].append(dict(path=occ.key,part=part.key,visual=visible,collision=bool(collide)))
    if not visual.geometry: raise ValueError('stage produced no visual geometry')
    if collision is not None and not collision.geometry: raise ValueError('selected collision policy produced no geometry')
    lighting=convert_lights(ctx,occurrences)
    bounds=visual.bounds.copy()
    if collision is not None:
        bounds[0]=np.minimum(bounds[0],collision.bounds[0]); bounds[1]=np.maximum(bounds[1],collision.bounds[1])
    if not np.isfinite(bounds).all(): raise ValueError('scene bounds are not finite')
    def triangles(scene):
        return sum(len(scene.geometry[scene.graph[n][1]].faces) for n in scene.graph.nodes_geometry) if scene is not None else 0
    ctx.report['counts'].update(visual_triangles=triangles(visual),collision_triangles=triangles(collision),
          geometry_count=len(visual.geometry),material_count=len({_material_digest(m) for m in visual.geometry.values() if m.visual.kind=='texture'}),
          occurrence_count=len(visual.graph.nodes_geometry))
    ctx.report.update(bounds_min=bounds[0].tolist(),bounds_max=bounds[1].tolist(),
                      lights=dict(local=lighting.local,sun=lighting.sun,defaulted=lighting.defaulted))
    return Bundle(visual,collision,lighting,bounds,ctx.report)


def export_glb(scene,path):
    copy=trimesh.Scene(base_frame=scene.graph.base_frame)
    copy.graph=scene.graph.copy()
    copy.geometry={name:mesh.copy(include_cache=True) for name,mesh in scene.geometry.items()}
    copy.apply_transform(trimesh.transformations.rotation_matrix(-np.pi/2,[1,0,0]))
    overrides={m.visual.material.name:m.metadata.get('usd_gltf_overrides',{})
               for m in copy.geometry.values() if m.visual.kind=='texture'}

    def postprocess(tree):
        for material in tree.get('materials',[]):
            extra=overrides.get(material.get('name'),{})
            if extra.get('extensions'):
                material.setdefault('extensions',{}).update(extra['extensions'])
                tree['extensionsUsed']=sorted(set(tree.get('extensionsUsed',[]))|set(extra['extensions']))
            for slot,sampler in extra.get('samplers',{}).items():
                holder=material.get('pbrMetallicRoughness',{}) if slot in ('baseColorTexture','metallicRoughnessTexture') else material
                if slot not in holder: continue
                texture=tree['textures'][holder[slot]['index']]
                samplers=tree.setdefault('samplers',[])
                if sampler not in samplers: samplers.append(sampler)
                texture['sampler']=samplers.index(sampler)
    try:
        data=trimesh.exchange.gltf.export_glb(copy,include_normals=True,tree_postprocessor=postprocess)
    except (struct.error,OverflowError) as error:
        raise ValueError('GLB size exceeds format limits; reduce geometry/texture resolution or selected roots') from error
    if len(data)>0xffffffff: raise ValueError('GLB size exceeds 4 GiB format limit; select smaller roots')
    Path(path).write_bytes(data)
