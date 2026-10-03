"""Assemble reusable scene geometry and export ARGoS environment bundles."""
from dataclasses import dataclass
from pathlib import Path
import hashlib
import json
import struct
import tempfile
import shutil
import sys
import math
import xml.etree.ElementTree as ET
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


def _destination(out,name,overwrite):
    destination=Path(out).resolve()/name
    if destination.is_symlink(): raise ValueError(f'destination is a symlink: {destination}')
    if destination.exists():
        if not overwrite: raise ValueError(f'bundle exists: {destination}; use --overwrite')
        try:
            report=json.loads((destination/(name+'.json')).read_text())
            if report.get('importer')!='openusd-static': raise ValueError('not an importer bundle')
        except (OSError,ValueError) as error:
            raise ValueError(f'refusing to overwrite unrelated directory: {destination}') from error
    return destination


def _fmt(values):
    return ','.join(format(float(v),'.9g') for v in values)


def _xml(path,root):
    ET.indent(root,space='  ')
    ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)


def write_bundle(bundle,ctx):
    name=ctx.name; destination=_destination(ctx.options.out,name,ctx.options.overwrite)
    out=destination.parent; out.mkdir(parents=True,exist_ok=True)
    temporary=Path(tempfile.mkdtemp(prefix='.'+name+'.tmp-',dir=out)); backup=None
    try:
        export_glb(bundle.visual,temporary/(name+'.glb'))
        if bundle.collision is not None: export_glb(bundle.collision,temporary/(name+'.collision.glb'))
        # Validate actual exported assets before replacing any existing bundle.
        for path in temporary.glob('*.glb'):
            loaded=trimesh.load(path,force='scene')
            if not loaded.geometry or not np.isfinite(loaded.bounds).all(): raise ValueError(f'invalid exported GLB: {path.name}')
        lights=ET.Element('lights')
        for light in bundle.lighting.local:
            attributes={k:_fmt(light[k]) for k in ('position','color')}
            attributes.update(intensity=str(light['intensity']),falloff=str(light['falloff']),cast_shadows=str(light['cast_shadows']).lower())
            if light['type']=='spot':
                attributes.update(direction=_fmt(light['direction']),inner_angle=str(light['inner_angle']),outer_angle=str(light['outer_angle']))
            ET.SubElement(lights,light['type'],attributes)
        _xml(temporary/(name+'.lights.xml'),lights)
        lower,upper=bundle.bounds; center=(lower+upper)/2; diagonal=float(np.linalg.norm(upper-lower))
        camera=center+np.array([max(diagonal*.65,2),-max(diagonal*.65,2),max(diagonal*.4,1)])
        viewer=dict(position=_fmt(camera),look_at=_fmt(center),resolution='1280,720',near='0.1',far=str(max(300,2*diagonal)))
        debug=dict(position=_fmt(camera),look_at=_fmt(center),fov='70',resolution='960,540',period='10',dump='frames')
        try:
            stops=ctx.options.stops; aperture=round(16/2**(min(stops,6)/2),1)
            remaining=max(0,stops-6); shutter=round(.01*2**min(remaining,2),4); iso=round(100*2**max(0,remaining-2))
        except (OverflowError,ZeroDivisionError) as error: raise ValueError('exposure stops outside representable range') from error
        if not math.isfinite(aperture) or aperture<=0: raise ValueError('invalid aperture')
        root=ET.Element('argos-configuration'); framework=ET.SubElement(root,'framework')
        ET.SubElement(framework,'system',threads='0'); ET.SubElement(framework,'experiment',length='0',ticks_per_second='10',random_seed='1')
        ET.SubElement(root,'controllers')
        arena=ET.SubElement(root,'arena',size=_fmt(np.maximum(upper-lower+4,.001)),center=_fmt(center))
        if bundle.collision is not None:
            ET.SubElement(arena,'mesh',id='world',file=name+'.collision.glb',y_up='true',double_sided='true')
        jolt=ET.SubElement(ET.SubElement(root,'physics_engines'),'jolt',id='jolt'); ET.SubElement(jolt,'gravity',g='9.81')
        medium=ET.SubElement(ET.SubElement(root,'media'),'photorealism',id='pr',backend='vulkan',draw_floor='false')
        sun=bundle.lighting.sun or dict(direction=[0,0,-1],intensity=0,cast_shadows=False)
        ET.SubElement(medium,'sun',direction=_fmt(sun['direction']),intensity=str(sun['intensity']),cast_shadows=str(sun['cast_shadows']).lower())
        ET.SubElement(medium,'skybox',color='0.53,0.71,0.92' if bundle.lighting.defaulted else '0,0,0')
        ET.SubElement(medium,'exposure',aperture=str(aperture),shutter_speed=str(shutter),sensitivity=str(iso))
        medium.append(lights)
        ET.SubElement(ET.SubElement(medium,'scenery'),'prop',model=name+'.glb',position='0,0,0',orientation='0,0,90',scale='1')
        ET.SubElement(medium,'debug_camera',debug)
        ET.SubElement(ET.SubElement(root,'visualization'),'filament',viewer,medium='pr',speed='1',flashlight='true')
        _xml(temporary/(name+'.argos'),root)
        ctx.report.update(importer='openusd-static',name=name,camera=dict(debug=debug,viewer=viewer),
                          outputs=[p.name for p in sorted(temporary.iterdir())]+[name+'.json'])
        (temporary/(name+'.json')).write_text(json.dumps(ctx.report,indent=2,sort_keys=True,allow_nan=False)+'\n')
        # Recheck just before publication, including symlinks and ownership marker.
        _destination(out,name,ctx.options.overwrite)
        if destination.exists():
            backup=Path(tempfile.mkdtemp(prefix='.'+name+'.backup-',dir=out)); backup.rmdir()
            destination.rename(backup)
        try:
            temporary.rename(destination)
        except BaseException as error:
            if backup is not None and backup.exists():
                try: backup.rename(destination)
                except OSError as restore_error:
                    raise OSError(f'publication failed ({error}); rollback failed ({restore_error}); old bundle retained at {backup}') from error
            raise
        if backup is not None:
            try: shutil.rmtree(backup)
            except OSError as error: print(f'warning: bundle published; old backup retained at {backup}: {error}',file=sys.stderr)
        return destination
    finally:
        if temporary.exists(): shutil.rmtree(temporary)
