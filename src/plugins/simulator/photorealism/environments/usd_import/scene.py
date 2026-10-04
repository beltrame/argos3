"""Compose one USD snapshot and enumerate its placed geometry and lights."""
from dataclasses import dataclass, field, asdict
from pathlib import Path
import math
import re
import importlib.metadata
import numpy as np
from pxr import Usd, UsdGeom, UsdLux, UsdPhysics, Sdf


@dataclass
class Options:
    source: Path
    out: Path = Path('.')
    name: str | None = None
    time: float | None = None
    roots: tuple[str, ...] = ()
    purpose: str = 'render'
    collision: str = 'auto'
    texture_size: int = 1024
    light_scale: float = 1.0
    light_range: float = 20.0
    stops: float = 6.0
    strict: bool = False
    overwrite: bool = False


@dataclass
class SceneContext:
    stage: Usd.Stage
    options: Options
    name: str
    time: Usd.TimeCode
    stage_to_argos: np.ndarray
    report: dict
    cache: dict = field(default_factory=dict)


@dataclass
class Occurrence:
    prim: Usd.Prim
    key: str
    transform: np.ndarray


def warn(ctx, code, path, message, **details):
    ctx.report['diagnostics'].append(dict(code=code, path=str(path), message=message, details=details))
    if ctx.options.strict:
        raise ValueError(f'{path}: {message} ({code})')


def read_stage(options):
    source = Path(options.source).resolve()
    if source.suffix.lower() not in ('.usd', '.usda', '.usdc'):
        raise ValueError('input must be a local .usd, .usda or .usdc stage')
    name = options.name
    if name is None:
        name = re.sub('[^a-z0-9]+', '_', source.stem.lower()).strip('_') or 'scene'
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*', name):
        raise ValueError('name must match [A-Za-z0-9][A-Za-z0-9_-]*')
    if options.purpose not in ('render', 'proxy') or options.collision not in ('auto','authored','visual','none'):
        raise ValueError('invalid purpose or collision policy')
    if not isinstance(options.texture_size, int) or options.texture_size < 0:
        raise ValueError('texture size must be a nonnegative integer')
    for key in ('time', 'light_scale', 'light_range', 'stops'):
        value = getattr(options, key)
        if value is not None and not math.isfinite(value):
            raise ValueError(f'{key} must be finite')
    if options.light_scale <= 0 or options.light_range <= 0:
        raise ValueError('light scale and range must be positive')
    try:
        stage = Usd.Stage.Open(str(source), load=Usd.Stage.LoadAll)
    except Exception as error:
        raise ValueError(f'cannot open {source}: {error}') from error
    if not stage:
        raise ValueError(f'cannot open {source}')
    errors = stage.GetCompositionErrors()
    if errors:
        raise ValueError(f'{source}: composition errors: ' + '; '.join(map(str, errors)))
    for root in options.roots:
        path = Sdf.Path(root)
        if not path.IsAbsoluteRootOrPrimPath() or not stage.GetPrimAtPath(path):
            raise ValueError(f'invalid or missing root: {root}')
    axis = str(UsdGeom.GetStageUpAxis(stage))
    units = float(UsdGeom.GetStageMetersPerUnit(stage))
    if axis not in ('Y','Z') or not math.isfinite(units) or units <= 0:
        raise ValueError('unsupported up axis or invalid metersPerUnit')
    transform = np.eye(4)
    if axis == 'Y':
        transform[:3,:3] = [[1,0,0],[0,0,-1],[0,1,0]]
    transform[:3,:3] *= units
    time = Usd.TimeCode(options.time) if options.time is not None else (
        Usd.TimeCode(stage.GetStartTimeCode()) if stage.HasAuthoredTimeCodeRange() else Usd.TimeCode.Default())
    settings = asdict(options)
    settings.update(source=str(source), out=str(options.out), roots=list(options.roots))
    report = dict(source=str(source), options=settings,
                  versions={k:importlib.metadata.version(k) for k in ('usd-core','numpy','trimesh','Pillow')},
                  time=None if time.IsDefault() else time.GetValue(),
                  metadata=dict(up_axis=axis, meters_per_unit=units), counts={'point_instances':0},
                  diagnostics=[], sources=[])
    return SceneContext(stage, options, name, time, transform, report)


def _matrix(value, path):
    matrix = np.asarray(value, dtype=float).T.copy()
    if not np.isfinite(matrix).all() or abs(np.linalg.det(matrix[:3,:3])) < 1e-15:
        raise ValueError(f'{path}: non-finite or singular transform')
    return matrix


def _renderable(prim):
    return bool(UsdGeom.Gprim(prim) or UsdLux.LightAPI(prim))


def iter_occurrences(ctx):
    stage = ctx.stage
    cache = UsdGeom.XformCache(ctx.time)
    predicate = Usd.TraverseInstanceProxies()
    prims = list(stage.Traverse(predicate))
    prototype_paths = set()
    for prim in prims:
        if prim.IsA(UsdGeom.PointInstancer):
            prototype_paths.update(UsdGeom.PointInstancer(prim).GetPrototypesRel().GetTargets())
    roots = [Sdf.Path(p) for p in ctx.options.roots]
    result = []
    ctx.report['counts']['point_instances'] = 0
    visibility = ctx.cache.setdefault('visibility', {})
    purposes = ctx.cache.setdefault('purpose', {})

    def selected(path):
        return not roots or any(path.HasPrefix(r) for r in roots)

    def prototype_purpose(child, proto, inherited):
        while child and child.GetPath().HasPrefix(proto.GetPath()):
            attr=UsdGeom.Imageable(child).GetPurposeAttr()
            if attr and attr.HasAuthoredValueOpinion(): return str(attr.Get(ctx.time))
            child=child.GetParent()
        return inherited

    def emit(prim, key, transform, visible=True, purpose=None):
        if _renderable(prim):
            if not np.isfinite(transform).all() or abs(np.linalg.det(transform[:3,:3])) < 1e-20:
                raise ValueError(f'{key}: non-finite or singular transform')
            result.append(Occurrence(prim, key, transform))
            visibility[key] = visible
            purposes[key] = purpose if purpose is not None else UsdGeom.Imageable(prim).ComputePurpose()

    def expand(prim, key, world, active=(), visible=True, purpose=None):
        if str(prim.GetPath()) in active:
            raise ValueError(f'{key}: cyclic point-instancer prototypes')
        inst = UsdGeom.PointInstancer(prim)
        targets = inst.GetPrototypesRel().GetTargets()
        indices = list(inst.GetProtoIndicesAttr().Get(ctx.time) or [])
        positions = inst.GetPositionsAttr().Get(ctx.time)
        if positions is None or len(positions) != len(indices) or any(i < 0 or i >= len(targets) for i in indices):
            raise ValueError(f'{key}: invalid point-instancer positions/prototype indices')
        if not indices:
            return
        matrices = inst.ComputeInstanceTransformsAtTime(ctx.time, ctx.time,
                      UsdGeom.PointInstancer.IncludeProtoXform, UsdGeom.PointInstancer.IgnoreMask)
        mask = list(inst.ComputeMaskAtTime(ctx.time))
        if len(matrices) != len(indices) or (mask and len(mask) != len(indices)):
            raise ValueError(f'{key}: invalid instance transforms/mask')
        visible = visible and UsdGeom.Imageable(prim).ComputeVisibility(ctx.time) != 'invisible'
        if purpose is None: purpose=UsdGeom.Imageable(prim).ComputePurpose()
        for i, proto_index in enumerate(indices):
            if mask and not mask[i]:
                continue
            proto = stage.GetPrimAtPath(targets[proto_index])
            if not proto:
                raise ValueError(f'{key}: missing prototype {targets[proto_index]}')
            placement = world @ _matrix(matrices[i], key)
            inverse = np.linalg.inv(_matrix(cache.GetLocalToWorldTransform(proto), str(proto.GetPath())))
            ctx.report['counts']['point_instances'] += 1
            descendants = list(Usd.PrimRange(proto, predicate))
            nested_targets = {p for child in descendants if child.IsA(UsdGeom.PointInstancer)
                              for p in UsdGeom.PointInstancer(child).GetPrototypesRel().GetTargets()}
            for child in descendants:
                if (any(child.GetPath().HasPrefix(p) for p in nested_targets)
                        or not (_renderable(child) or child.IsA(UsdGeom.PointInstancer))):
                    continue
                relative = inverse @ _matrix(cache.GetLocalToWorldTransform(child), str(child.GetPath()))
                child_key = f'{key}[{i}]{str(child.GetPath())[len(str(proto.GetPath())):]}'
                transform = placement @ relative
                if child.IsA(UsdGeom.PointInstancer):
                    expand(child,child_key,transform,active+(str(prim.GetPath()),),visible,prototype_purpose(child,proto,purpose))
                else:
                    emit(child,child_key,transform,visible,prototype_purpose(child,proto,purpose))

    for prim in prims:
        path = prim.GetPath()
        if (not selected(path) or any(path.HasPrefix(p) for p in prototype_paths)
                or not (_renderable(prim) or prim.IsA(UsdGeom.PointInstancer))):
            continue
        world = ctx.stage_to_argos @ _matrix(cache.GetLocalToWorldTransform(prim), str(path))
        if prim.IsA(UsdGeom.PointInstancer):
            expand(prim,str(path),world)
        else:
            emit(prim,str(path),world)
    yield from sorted(result, key=lambda item:item.key)


def is_visual(occ, ctx):
    imageable = UsdGeom.Imageable(occ.prim)
    return (ctx.cache.get('visibility',{}).get(occ.key,True)
            and imageable.ComputeVisibility(ctx.time) != 'invisible'
            and (ctx.cache.get('purpose',{}).get(occ.key) or imageable.ComputePurpose()) in ('default',ctx.options.purpose))


def is_collider(occ, ctx):
    return (occ.prim.HasAPI(UsdPhysics.CollisionAPI)
            and bool(UsdPhysics.CollisionAPI(occ.prim).GetCollisionEnabledAttr().Get(ctx.time)))
