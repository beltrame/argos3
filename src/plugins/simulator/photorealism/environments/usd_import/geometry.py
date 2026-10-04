"""Triangulate USD faces while retaining corner attributes and face bindings."""
from dataclasses import dataclass
import numpy as np
import trimesh
from pxr import UsdGeom, UsdSkel
from .scene import warn, is_collider


@dataclass
class GeometryPart:
    key: str
    binding_prim: object
    mesh: trimesh.Trimesh
    uvs: dict


def _cross(a,b):
    return a[0]*b[1]-a[1]*b[0]


def _triangulate(points, path, ctx=None):
    if len(points) < 3:
        raise ValueError(f'{path}: face has fewer than three corners')
    points=points-points[0]
    normal=np.cross(points,np.roll(points,-1,axis=0)).sum(axis=0)
    length=np.linalg.norm(normal)
    axes=[int(np.argmax(np.abs(normal))) if length>=1e-14 else int(np.argmin(np.ptp(points,axis=0)))]
    # A twisted but valid 3D cage can cross itself in its dominant projection.
    nonplanar=length>=1e-14 and np.max(np.abs(points @ normal)) > np.ptp(points,axis=0).max()*length*np.finfo(float).eps*32
    if nonplanar:
        axes.extend(a for a in np.argsort(-np.abs(normal)) if a!=axes[0] and abs(normal[a])>length*1e-12)
    for axis in axes:
        try: return _triangulate_projected(np.delete(points,axis,axis=1),path)
        except ValueError as error: failure=error
    if len(points)==4 and nonplanar:
        tolerance=np.linalg.norm(np.ptp(points,axis=0))*np.finfo(float).eps*32
        for a,b,c,d in ((0,1,2,3),(1,2,3,0)):
            edges=np.column_stack((points[b]-points[a],points[c]-points[d]))
            uv=np.linalg.lstsq(edges,points[c]-points[a],rcond=None)[0]
            if np.all((uv>=0)&(uv<=1)) and np.linalg.norm(edges @ uv-(points[c]-points[a]))<=tolerance:
                raise ValueError(f'{path}: self-intersecting 3D quad')
        if ctx is not None: warn(ctx,'twisted_quad',path,'nonplanar quad triangulated along shorter 3D diagonal')
        return ([[0,1,2],[0,2,3]] if np.linalg.norm(points[2]-points[0])<=np.linalg.norm(points[3]-points[1])
                else [[0,1,3],[1,2,3]])
    raise failure


def _triangulate_projected(p,path):
    n=len(p)
    epsilon = max(float(np.ptp(p,axis=0).max())**2*1e-12,1e-20)
    # ponytail: O(n²) per polygon; use a tessellation library if very large n-gons dominate.
    for i in range(n):
        a,b=p[i],p[(i+1)%n]
        for j in range(i+2,n):
            if i==0 and j==n-1: continue
            c,d=p[j],p[(j+1)%n]
            if (_cross(b-a,c-a)*_cross(b-a,d-a) < -epsilon**2
                    and _cross(d-c,a-c)*_cross(d-c,b-c) < -epsilon**2):
                raise ValueError(f'{path}: self-intersecting polygon')
    area = sum(_cross(p[i],p[(i+1)%n]) for i in range(n))
    if abs(area) <= epsilon:
        return []
    sign = 1 if area>0 else -1
    remaining=list(range(n)); triangles=[]
    while len(remaining)>3:
        for k,b in enumerate(remaining):
            a,c=remaining[k-1],remaining[(k+1)%len(remaining)]
            if sign*_cross(p[b]-p[a],p[c]-p[b]) <= epsilon:
                continue
            inside=False
            for j in remaining:
                if j in (a,b,c): continue
                if all(sign*_cross(p[y]-p[x],p[j]-p[x]) >= -epsilon for x,y in ((a,b),(b,c),(c,a))):
                    inside=True; break
            if not inside:
                triangles.append([a,b,c]); remaining.pop(k); break
        else:
            raise ValueError(f'{path}: cannot triangulate polygon (repeated/collinear corners)')
    triangles.append(remaining)
    return triangles


def _expanded(values, interpolation, vertices, face_ids, corners, counts):
    values=np.asarray(values)
    if interpolation=='constant':
        if len(values)!=1: raise ValueError('constant attribute needs one value')
        return np.repeat(values,len(vertices),axis=0)
    if interpolation=='uniform':
        if len(values)!=len(counts): raise ValueError('uniform attribute cardinality')
        return values[face_ids]
    if interpolation in ('vertex','varying'):
        if len(values)<=int(np.max(vertices)): raise ValueError('vertex attribute cardinality')
        return values[vertices]
    if interpolation=='faceVarying':
        if len(values)!=int(np.sum(counts)): raise ValueError('corner attribute cardinality')
        return values[corners]
    raise ValueError(f'unsupported interpolation {interpolation}')


def _primitive(prim, time):
    if prim.IsA(UsdGeom.Cube):
        size=UsdGeom.Cube(prim).GetSizeAttr().Get(time)
        mesh=trimesh.creation.box(extents=[size]*3)
    elif prim.IsA(UsdGeom.Sphere):
        mesh=trimesh.creation.uv_sphere(radius=UsdGeom.Sphere(prim).GetRadiusAttr().Get(time),count=[24,32])
    else:
        schema=next((c for c in (UsdGeom.Cylinder,UsdGeom.Cone,UsdGeom.Capsule) if prim.IsA(c)),None)
        if schema is None: return None
        shape=schema(prim); radius=shape.GetRadiusAttr().Get(time); height=shape.GetHeightAttr().Get(time)
        if not np.isfinite([radius,height]).all() or radius<=0 or height<=0:
            raise ValueError(f'{prim.GetPath()}: invalid primitive dimensions')
        if schema==UsdGeom.Capsule:
            mesh=trimesh.creation.capsule(radius=radius,height=height)
        elif schema==UsdGeom.Cylinder:
            mesh=trimesh.creation.cylinder(radius=radius,height=height,sections=32)
        else:
            mesh=trimesh.creation.cone(radius=radius,height=height,sections=32)
            mesh.apply_translation([0,0,-height/2])
        axis=shape.GetAxisAttr().Get(time)
        rotation={'X':[[0,0,1],[0,1,0],[-1,0,0]],'Y':[[1,0,0],[0,0,1],[0,-1,0]],'Z':np.eye(3)}
        if axis not in rotation: raise ValueError(f'{prim.GetPath()}: invalid primitive axis')
        matrix=np.eye(4); matrix[:3,:3]=rotation[axis]; mesh.apply_transform(matrix)
    if not np.isfinite(mesh.vertices).all() or np.any(mesh.extents<=0):
        raise ValueError(f'{prim.GetPath()}: invalid primitive dimensions')
    return mesh


def convert_geometry(occ,ctx):
    prim=occ.prim; time=ctx.time; path=occ.key
    if not np.isfinite(occ.transform).all() or abs(np.linalg.det(occ.transform[:3,:3]))<1e-20:
        raise ValueError(f'{path}: invalid transform')
    if prim.HasAPI(UsdSkel.BindingAPI):
        warn(ctx,'skinning',path,'skinning is not evaluated; authored points only')
    source=UsdGeom.Mesh(prim)
    if source:
        points=np.asarray(source.GetPointsAttr().Get(time),dtype=float)
        counts=np.asarray(source.GetFaceVertexCountsAttr().Get(time),dtype=int)
        indices=np.asarray(source.GetFaceVertexIndicesAttr().Get(time),dtype=int)
        if (points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all()
                or counts.ndim!=1 or indices.ndim!=1 or np.any(counts<3)
                or counts.sum()!=len(indices) or np.any(indices<0) or np.any(indices>=len(points))):
            raise ValueError(f'{path}: invalid mesh topology/points')
        if source.GetSubdivisionSchemeAttr().Get(time)!='none':
            warn(ctx,'subdivision',path,'subdivision exported as control cage')
        holes=set(source.GetHoleIndicesAttr().Get(time) or [])
        if any(i<0 or i>=len(counts) for i in holes): raise ValueError(f'{path}: invalid hole face')
        triangles=[]; face_ids=[]; offset=0; dropped=0
        for face,n in enumerate(counts):
            if face not in holes:
                local=_triangulate(points[indices[offset:offset+n]],path,ctx)
                if not local: dropped+=1
                for tri in local:
                    corners=np.asarray(tri)+offset
                    xyz=points[indices[corners]]
                    if np.linalg.norm(np.cross(xyz[1]-xyz[0],xyz[2]-xyz[0]))<1e-16:
                        dropped+=1; continue
                    triangles.append(corners); face_ids.append(face)
            offset+=n
        if dropped: warn(ctx,'degenerate',path,'zero-area faces/triangles dropped',count=dropped)
        if not triangles: return []
        triangle_corners=np.array(triangles)
        if source.GetOrientationAttr().Get(time)=='leftHanded': triangle_corners=triangle_corners[:,::-1]
        corners=triangle_corners.ravel(); vertices=indices[corners]
        corner_faces=np.repeat(face_ids,3)
        xyz=points[vertices]
        faces=np.arange(len(xyz)).reshape(-1,3)
    else:
        mesh=_primitive(prim,time)
        if mesh is None:
            if is_collider(occ,ctx): raise ValueError(f'{path}: unsupported authored collider {prim.GetTypeName()}')
            warn(ctx,'geometry',path,f'unsupported geometry {prim.GetTypeName()}')
            return []
        xyz=mesh.vertices[mesh.faces.ravel()]; faces=np.arange(len(xyz)).reshape(-1,3)
        vertices=mesh.faces.ravel(); corners=np.arange(len(xyz)); face_ids=list(range(len(faces)))
        counts=np.full(len(faces),3); corner_faces=np.repeat(face_ids,3)

    api=UsdGeom.PrimvarsAPI(prim)
    attributes={p.GetPrimvarName():p for p in api.FindPrimvarsWithInheritance()}
    uvs={}; colors=None; opacity=None; normals=None
    for name,pv in attributes.items():
        value=pv.ComputeFlattened(time)
        if value is None: continue
        a=np.asarray(value)
        if not a.size: continue
        if not pv.GetTypeName().isArray: a=a.reshape((1,)+a.shape)
        useful=name in ('displayColor','displayOpacity','normals') or (a.ndim==2 and a.shape[1]==2)
        if not useful: continue
        try:
            expanded=_expanded(a,str(pv.GetInterpolation()),vertices,corner_faces,corners,counts)
            if not np.isfinite(expanded).all(): raise ValueError('non-finite attribute')
        except (ValueError,IndexError) as error:
            warn(ctx,'attribute',path,f'{name}: {error}'); continue
        if name=='displayColor': colors=expanded
        elif name=='displayOpacity': opacity=expanded
        elif name=='normals': normals=expanded
        else: uvs[name]=expanded.astype(float)
    if normals is None and source:
        value=source.GetNormalsAttr().Get(time)
        if value is not None and len(value):
            try: normals=_expanded(value,str(source.GetNormalsInterpolation()),vertices,corner_faces,corners,counts)
            except (ValueError,IndexError) as error: warn(ctx,'normals',path,f'invalid authored normals: {error}')
    if normals is not None:
        length=np.linalg.norm(normals,axis=1) if normals.shape==(len(xyz),3) else np.zeros(len(xyz))
        if not np.isfinite(normals).all() or np.any(length<1e-12):
            warn(ctx,'normals',path,'invalid normals regenerated'); normals=None
        else: normals=normals/length[:,None]
    groups=[]; assigned=set()
    if source:
        for subset in UsdGeom.Subset.GetGeomSubsets(UsdGeom.Imageable(source),'face','materialBind'):
            selected=set(subset.GetIndicesAttr().Get(time) or [])
            if selected & assigned or any(i<0 or i>=len(counts) for i in selected):
                raise ValueError(f'{path}: invalid/overlapping material subset')
            assigned.update(selected); groups.append((subset.GetPrim(),selected))
    groups.append((prim,set(face_ids)-assigned))
    parts=[]
    for binding,selected in groups:
        chosen=np.flatnonzero(np.isin(face_ids,list(selected)))
        if not len(chosen): continue
        pick=faces[chosen].ravel()
        local=trimesh.Trimesh(vertices=xyz[pick],faces=np.arange(len(pick)).reshape(-1,3),process=False)
        local.vertex_normals=normals[pick] if normals is not None else np.repeat(local.face_normals,3,axis=0)
        if colors is not None or opacity is not None:
            rgba=np.ones((len(pick),4)); rgba[:,:3]=colors[pick] if colors is not None else 0.7
            if opacity is not None: rgba[:,3]=np.asarray(opacity[pick]).ravel()
            local.visual=trimesh.visual.ColorVisuals(local,vertex_colors=np.round(np.clip(rgba,0,1)*255).astype(np.uint8))
            local.metadata['usd_display_color']=True
        local.metadata['usd_double_sided']=bool(UsdGeom.Gprim(prim).GetDoubleSidedAttr().Get(time))
        parts.append(GeometryPart(str(binding.GetPath()),binding,local,{k:v[pick] for k,v in uvs.items()}))
    return parts
