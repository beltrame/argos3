"""Approximate composed USD lights with Filament's sun and local emitters."""
from dataclasses import dataclass
import math
import numpy as np
from pxr import UsdLux
from .scene import warn, is_visual


@dataclass
class Lighting:
    local: list
    sun: dict | None
    defaulted: bool


def convert_lights(ctx,occurrences):
    local=[]; distant=[]
    for occ in occurrences:
        prim=occ.prim; api=UsdLux.LightAPI(prim)
        if not api or not is_visual(occ,ctx): continue
        path=occ.key
        if prim.IsA(UsdLux.DomeLight):
            warn(ctx,'dome_light',path,'dome light/environment texture not converted')
            continue
        supported=any(prim.IsA(c) for c in (UsdLux.SphereLight,UsdLux.RectLight,UsdLux.DiskLight,UsdLux.CylinderLight,UsdLux.DistantLight))
        if not supported:
            warn(ctx,'light',path,f'unsupported light {prim.GetTypeName()}'); continue
        intensity=float(api.GetIntensityAttr().Get(ctx.time))
        exposure=float(api.GetExposureAttr().Get(ctx.time))
        if not math.isfinite(intensity) or intensity<0 or not math.isfinite(exposure):
            raise ValueError(f'{path}: invalid light intensity/exposure')
        try: intensity *= 2.0**exposure * ctx.options.light_scale
        except OverflowError as error: raise ValueError(f'{path}: light exposure overflow') from error
        if not math.isfinite(intensity): raise ValueError(f'{path}: light intensity overflow')
        if intensity==0: continue
        color=np.asarray(api.GetColorAttr().Get(ctx.time),dtype=float)
        if bool(api.GetEnableColorTemperatureAttr().Get(ctx.time)):
            temperature=float(api.GetColorTemperatureAttr().Get(ctx.time))
            if not math.isfinite(temperature) or temperature<=0: raise ValueError(f'{path}: invalid color temperature')
            color *= np.asarray(UsdLux.BlackbodyTemperatureAsRgb(temperature))
        if not np.isfinite(color).all() or np.any(color<0): raise ValueError(f'{path}: invalid light color')
        direction=occ.transform[:3,:3] @ [0,0,-1]
        length=np.linalg.norm(direction)
        if not math.isfinite(length) or length<1e-12: raise ValueError(f'{path}: invalid light direction')
        direction=(direction/length).tolist()
        shadows=UsdLux.ShadowAPI(prim)
        cast_shadows=bool(shadows.GetShadowEnableAttr().Get(ctx.time)) if prim.HasAPI(UsdLux.ShadowAPI) else False
        for rel in prim.GetRelationships():
            if rel.GetTargets() and ('lightLink' in rel.GetName() or 'shadowLink' in rel.GetName() or rel.GetName()=='filters'):
                warn(ctx,'light_linking',path,'light/shadow linking and filters are not converted',relationship=rel.GetName())
        for attr in prim.GetAttributes():
            if ('lightLink' in attr.GetName() or 'shadowLink' in attr.GetName()) and attr.HasAuthoredValueOpinion():
                warn(ctx,'light_linking',path,'authored light/shadow collection settings are not converted',attribute=attr.GetName())
        normalize=bool(api.GetNormalizeAttr().Get(ctx.time))
        if normalize: warn(ctx,'light_normalization',path,'photometric intensity proxy ignores USD normalization',normalize=normalize)
        if prim.IsA(UsdLux.DistantLight):
            distant.append(dict(source=path,direction=direction,intensity=intensity,cast_shadows=cast_shadows,color=color.tolist()))
            if not np.allclose(color,1): warn(ctx,'sun_color',path,'photorealism sun has fixed white color; authored sun color omitted',color=color.tolist())
            continue
        details={}
        for name in ('radius','width','height','length','treatAsPoint'):
            attr=prim.GetAttribute(name if name=='treatAsPoint' else 'inputs:'+name)
            if attr: details[name]=attr.Get(ctx.time)
        if not prim.IsA(UsdLux.SphereLight) or not details.get('treatAsPoint',False):
            warn(ctx,'area_light',path,'finite emitter approximated as a local point/spot light',**details)
        light=dict(source=path,type='point',position=occ.transform[:3,3].tolist(),color=color.tolist(),
                   intensity=intensity,falloff=ctx.options.light_range,cast_shadows=cast_shadows)
        if prim.HasAPI(UsdLux.ShapingAPI):
            shaping=UsdLux.ShapingAPI(prim)
            angle=float(shaping.GetShapingConeAngleAttr().Get(ctx.time))
            softness=float(shaping.GetShapingConeSoftnessAttr().Get(ctx.time))
            focus=float(shaping.GetShapingFocusAttr().Get(ctx.time))
            if not np.isfinite([angle,softness,focus]).all() or angle<0: raise ValueError(f'{path}: invalid shaping')
            outer=float(np.clip(angle,.1,89))
            if outer!=angle or softness!=0 or focus!=0:
                warn(ctx,'shaping',path,'cone clipping/softness/focus approximated',angle=angle,softness=softness,focus=focus)
            light.update(type='spot',direction=direction,inner_angle=outer,outer_angle=outer)
        local.append(light)
    distant.sort(key=lambda light:(-light['intensity'],light['source']))
    for omitted in distant[1:]: warn(ctx,'distant_light',omitted['source'],'only strongest distant light is supported')
    sun=distant[0] if distant else None
    defaulted=not local and sun is None
    if defaulted:
        sun=dict(source='generated',direction=[.6,.2,-1],intensity=100000,cast_shadows=True)
        warn(ctx,'generated_lighting','/','no usable lights; generated inspectable sun/sky illumination')
    return Lighting(local,sun,defaulted)
