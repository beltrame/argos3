"""USD Preview Surface's portable subset, baked into glTF PBR textures."""
from dataclasses import dataclass
from pathlib import Path
import glob
import math
import re
import numpy as np
from PIL import Image
from pxr import UsdShade, Sdf, UsdGeom
from trimesh.visual.material import PBRMaterial
from trimesh.visual import TextureVisuals
from .scene import warn


@dataclass
class MaterialData:
    pbr: PBRMaterial
    uv_name: str | None
    uv_transform: np.ndarray
    gltf_overrides: dict


def _linear(value):
    return np.where(value<=0.04045,value/12.92,((value+0.055)/1.055)**2.4)


def _srgb(value):
    value=np.maximum(value,0)
    return np.where(value<=0.0031308,12.92*value,1.055*value**(1/2.4)-0.055)


def _image(value, srgb=False):
    value=np.asarray(value,dtype=float)
    if srgb:
        value=value.copy(); value[:,:,:3]=_srgb(value[:,:,:3])
    return Image.fromarray(np.round(np.clip(value,0,1)*255).astype(np.uint8))


def _value(attr, default, time, visited=()):
    if not attr: return default
    path=str(attr.GetAttr().GetPath())
    if path in visited: raise ValueError(f'cyclic shader connection at {path}')
    source=attr.GetConnectedSource()
    if source:
        api,name,kind=source
        target=api.GetInput(name) if kind==UsdShade.AttributeType.Input else api.GetOutput(name)
        return _value(target,default,time,visited+(path,))
    value=attr.GetAttr().Get(time)
    return default if value is None else value


def _uv(attr,ctx,visited=()):
    if not attr or not attr.GetConnectedSource(): return 'st',np.eye(3)
    api,name,kind=attr.GetConnectedSource()
    path=str(api.GetPrim().GetPath())+'.'+name
    if path in visited: raise ValueError(f'cyclic UV graph at {path}')
    shader=UsdShade.Shader(api.GetPrim()); shader_id=shader.GetIdAttr().Get() if shader else None
    visited=visited+(path,)
    if shader_id=='UsdPrimvarReader_float2':
        return str(_value(shader.GetInput('varname'),'st',ctx.time)),np.eye(3)
    if shader_id=='UsdTransform2d':
        uv,matrix=_uv(shader.GetInput('in'),ctx,visited)
        scale=np.asarray(_value(shader.GetInput('scale'),(1,1),ctx.time),dtype=float)
        offset=np.asarray(_value(shader.GetInput('translation'),(0,0),ctx.time),dtype=float)
        angle=math.radians(float(_value(shader.GetInput('rotation'),0,ctx.time)))
        m=np.array([[math.cos(angle)*scale[0],-math.sin(angle)*scale[1],offset[0]],
                    [math.sin(angle)*scale[0],math.cos(angle)*scale[1],offset[1]],[0,0,1]])
        if not np.isfinite(m).all(): raise ValueError('non-finite UV transform')
        return uv,m @ matrix
    target=api.GetInput(name) if kind==UsdShade.AttributeType.Input else api.GetOutput(name)
    if not shader_id and target and target.GetConnectedSource(): return _uv(target,ctx,visited)
    raise ValueError(f'unsupported UV node {shader_id or path}')


def _asset(attr,ctx):
    asset=_value(attr,None,ctx.time)
    if asset is None: raise ValueError('texture has no file')
    path=asset.resolvedPath
    if not path:
        stack=attr.GetAttr().GetPropertyStack(ctx.time)
        layer=next((p.layer for p in stack if p.HasInfo('default')),ctx.stage.GetRootLayer())
        path=Sdf.ComputeAssetPathRelativeToLayer(layer,asset.path)
        if not Path(path).is_absolute():
            path=str(Path(layer.realPath).parent / path)
    if '<UDIM>' in path:
        files=[p for p in glob.glob(path.replace('<UDIM>','[0-9][0-9][0-9][0-9]')) if re.search(r'1\d{3}',Path(p).name)]
        if len(files)!=1: raise ValueError('multi-tile or missing UDIM textures are unsupported')
        tile=int(re.search(r'(1\d{3})',Path(files[0]).name).group(1))
        return files[0],tile
    return path,None


def _sample(attr,default,ctx,role,visited=()):
    if not attr or not attr.GetConnectedSource():
        value=np.asarray(_value(attr,default,ctx.time),dtype=float)
        if not np.isfinite(value).all(): raise ValueError('non-finite shader input')
        return value,None
    api,channel,kind=attr.GetConnectedSource()
    path=str(api.GetPrim().GetPath())+'.'+channel
    if path in visited: raise ValueError(f'cyclic shader graph at {path}')
    shader=UsdShade.Shader(api.GetPrim()); shader_id=shader.GetIdAttr().Get() if shader else None
    if shader_id!='UsdUVTexture':
        target=api.GetInput(channel) if kind==UsdShade.AttributeType.Input else api.GetOutput(channel)
        if not shader_id and target and target.GetConnectedSource():
            return _sample(target,default,ctx,role,visited+(path,))
        raise ValueError(f'unsupported shader node {shader_id or path}')
    try:
        filename,tile=_asset(shader.GetInput('file'),ctx)
        if Path(filename).suffix.lower() in ('.exr','.hdr'): raise ValueError('HDR/EXR texture decoding is unsupported')
        key=(str(Path(filename).resolve()),ctx.options.texture_size)
        images=ctx.cache.setdefault('images',{})
        if key not in images:
            with Image.open(filename) as original:
                image=original.convert('RGBA')
                if ctx.options.texture_size:
                    image.thumbnail((ctx.options.texture_size,ctx.options.texture_size),Image.Resampling.LANCZOS)
                images[key]=np.asarray(image,dtype=float)/255
        pixels=images[key].copy()
    except (OSError,ValueError) as error:
        warn(ctx,'texture',path,str(error))
        fallback=np.asarray(_value(shader.GetInput('fallback'),(0,0,0,1),ctx.time))
        pixels=np.broadcast_to(fallback,(1,1,4)).copy(); tile=None
    colorspace=str(_value(shader.GetInput('sourceColorSpace'),'auto',ctx.time))
    if colorspace=='sRGB' or (colorspace=='auto' and role in ('diffuseColor','emissiveColor')):
        pixels[:,:,:3]=_linear(pixels[:,:,:3])
    elif colorspace not in ('raw','auto'):
        raise ValueError(f'unsupported color space {colorspace}')
    scale=np.asarray(_value(shader.GetInput('scale'),(1,1,1,1),ctx.time))
    bias=np.asarray(_value(shader.GetInput('bias'),(0,0,0,0),ctx.time))
    pixels=pixels*scale+bias
    if not np.isfinite(pixels).all(): raise ValueError('non-finite texture values')
    channels={'r':0,'g':1,'b':2,'a':3,'rgb':slice(0,3)}
    if channel not in channels: raise ValueError(f'unsupported texture channel {channel}')
    uv,transform=_uv(shader.GetInput('st'),ctx)
    if tile is not None:
        transform=transform.copy(); transform[:2,2]-=[(tile-1001)%10,(tile-1001)//10]
    wraps={'repeat':10497,'clamp':33071,'mirror':33648,'useMetadata':10497}
    sampler={}
    for name in ('wrapS','wrapT'):
        value=str(_value(shader.GetInput(name),'useMetadata',ctx.time))
        if value not in wraps: raise ValueError(f'unsupported {name}={value}')
        sampler[name]=wraps[value]
    return pixels[:,:,channels[channel]],dict(uv=uv,transform=transform,sampler=sampler,tile=tile)


def convert_material(binding_prim,ctx):
    material,_=UsdShade.MaterialBindingAPI(binding_prim).ComputeBoundMaterial('preview')
    if not material: material,_=UsdShade.MaterialBindingAPI(binding_prim).ComputeBoundMaterial()
    source=material.ComputeSurfaceSource('preview')[0] if material else None
    if material and not source: source=material.ComputeSurfaceSource()[0]
    if not source or source.GetIdAttr().Get()!='UsdPreviewSurface':
        if material: warn(ctx,'shader',str(material.GetPath()),'no compatible USD Preview Surface; using display color')
        pbr=PBRMaterial(baseColorFactor=[1,1,1,1],roughnessFactor=.8,metallicFactor=0)
        return MaterialData(pbr,None,np.eye(3),{'display_fallback':True})
    key=(str(source.GetPath()),None if ctx.time.IsDefault() else ctx.time.GetValue())
    materials=ctx.cache.setdefault('materials',{})
    if key in materials: return materials[key]
    if bool(_value(source.GetInput('useSpecularWorkflow'),0,ctx.time)):
        warn(ctx,'shader',str(source.GetPath()),'specular workflow is unsupported; using display color')
        return MaterialData(PBRMaterial(baseColorFactor=[1,1,1,1]),None,np.eye(3),{'display_fallback':True})
    values={}; bindings={}; uv=None; transform=np.eye(3); overrides={'samplers':{},'extensions':{}}
    defaults={'diffuseColor':(.18,.18,.18),'metallic':0,'roughness':.5,'opacity':1,'emissiveColor':(0,0,0),'normal':(0,0,1)}
    for name,default in defaults.items():
        try:
            value,binding=_sample(source.GetInput(name),default,ctx,name)
            if binding:
                if uv is not None and (binding['uv']!=uv or not np.allclose(binding['transform'],transform)):
                    raise ValueError('incompatible texture UV primvars/transforms')
                uv=binding['uv']; transform=binding['transform']
                if binding.get('tile') is not None: overrides['single_udim']=True
            values[name]=value; bindings[name]=binding
        except ValueError as error:
            warn(ctx,'shader',str(source.GetPath()),f'{name}: {error}')
            values[name]=np.asarray(default); bindings[name]=None
    threshold=float(_value(source.GetInput('opacityThreshold'),0,ctx.time))
    if not math.isfinite(threshold): raise ValueError(f'{source.GetPath()}: non-finite alpha cutoff')
    pbr=PBRMaterial(baseColorFactor=[1,1,1,1],metallicFactor=1,roughnessFactor=1,
                    alphaMode='MASK' if threshold>0 else ('BLEND' if np.any(values['opacity']<1) else 'OPAQUE'),
                    alphaCutoff=float(np.clip(threshold,0,1)))

    def sized(value,shape,channels=None):
        if value.ndim<=1:
            return np.broadcast_to(value,shape+(channels,) if channels else shape).copy()
        if value.shape[:2]==shape: return value.copy()
        image=Image.fromarray(value.astype(np.float32)) if value.ndim==2 else _image(value)
        return np.asarray(image.resize((shape[1],shape[0]),Image.Resampling.BILINEAR),dtype=float)/(1 if value.ndim==2 else 255)

    def sampler(slot,*names):
        candidates=[bindings[n]['sampler'] for n in names if bindings[n]]
        if candidates:
            if any(c!=candidates[0] for c in candidates):
                warn(ctx,'texture',str(source.GetPath()),'combined texture wrap modes differ; first wrap mode used')
            overrides['samplers'][slot]=candidates[0]

    base=values['diffuseColor']; opacity=values['opacity']
    if bindings['diffuseColor'] or bindings['opacity']:
        shapes=[v.shape[:2] for v in (base,opacity) if v.ndim>=2]
        shape=max(shapes,key=lambda x:x[0]*x[1]); rgba=np.ones(shape+(4,))
        rgba[:,:,:3]=sized(base,shape,3); rgba[:,:,3]=sized(opacity,shape)
        pbr.baseColorTexture=_image(rgba,srgb=True); sampler('baseColorTexture','diffuseColor','opacity')
    else: pbr.baseColorFactor=np.r_[base,opacity]
    rough=values['roughness']; metal=values['metallic']
    if bindings['roughness'] or bindings['metallic']:
        shape=max([v.shape[:2] for v in (rough,metal) if v.ndim>=2],key=lambda x:x[0]*x[1])
        packed=np.ones(shape+(3,)); packed[:,:,1]=sized(rough,shape); packed[:,:,2]=sized(metal,shape)
        pbr.metallicRoughnessTexture=_image(packed); sampler('metallicRoughnessTexture','roughness','metallic')
    else: pbr.roughnessFactor=float(np.clip(rough,0,1)); pbr.metallicFactor=float(np.clip(metal,0,1))
    emission=values['emissiveColor']; strength=max(1,float(np.max(emission)))
    if strength>1: overrides['extensions']['KHR_materials_emissive_strength']={'emissiveStrength':strength}
    if bindings['emissiveColor']:
        pbr.emissiveTexture=_image(emission/strength,srgb=True); pbr.emissiveFactor=[1,1,1]
        sampler('emissiveTexture','emissiveColor')
    else: pbr.emissiveFactor=np.clip(emission/strength,0,1)
    if bindings['normal']:
        normal=values['normal']; length=np.linalg.norm(normal,axis=2,keepdims=True)
        if np.any(length<1e-12): warn(ctx,'normal',str(source.GetPath()),'zero normal texels replaced with +Z')
        normal=np.divide(normal,length,out=np.broadcast_to([0.,0.,1.],normal.shape).copy(),where=length>1e-12)
        pbr.normalTexture=_image(normal*.5+.5); sampler('normalTexture','normal')
    data=MaterialData(pbr,uv,transform,overrides); materials[key]=data
    return data


def attach_material(part,data,ctx):
    mesh=part.mesh.copy(); pbr=data.pbr.copy()
    pbr.doubleSided=mesh.metadata.get('usd_double_sided',False)
    if data.gltf_overrides.get('display_fallback'):
        if mesh.metadata.get('usd_display_color'):
            # Keep vertex colors (including corner discontinuities) in glTF.
            return mesh
        pbr.baseColorFactor=[.7,.7,.7,1]
    uv=None
    if data.uv_name:
        if data.uv_name not in part.uvs:
            warn(ctx,'uv',part.key,f'missing UV primvar {data.uv_name}; textures dropped')
            for name in ('baseColorTexture','metallicRoughnessTexture','emissiveTexture','normalTexture'):
                setattr(pbr,name,None)
        else:
            uv=(np.c_[part.uvs[data.uv_name],np.ones(len(mesh.vertices))] @ data.uv_transform.T)[:,:2]
            if data.gltf_overrides.get('single_udim') and (np.any(uv < -1e-6) or np.any(uv > 1+1e-6)):
                warn(ctx,'uv',part.key,'UVs extend outside the single resolved UDIM tile; textures dropped')
                for name in ('baseColorTexture','metallicRoughnessTexture','emissiveTexture','normalTexture'):
                    setattr(pbr,name,None)
    mesh.visual=TextureVisuals(uv=uv,material=pbr)
    mesh.metadata['usd_gltf_overrides']=data.gltf_overrides
    return mesh
