import io
import json
import struct
import unittest
from pathlib import Path
from unittest import mock
import numpy as np
import trimesh
from PIL import Image
from pxr import UsdGeom, UsdPhysics, UsdShade, UsdLux, Sdf
from support import author_mesh
from support import make_stage, context
from test_materials import fixture, texture
from usd_import.bundle import build_bundle, export_glb
from usd_import.geometry import convert_geometry


def glb_json(path):
    data=path.read_bytes(); size=struct.unpack_from('<I',data,12)[0]
    return json.loads(data[20:20+size])


class BundleTests(unittest.TestCase):
    def test_collision_modes_and_hidden_colliders(self):
        stage=make_stage(self)
        UsdGeom.Cube.Define(stage,'/Visible')
        hidden=UsdGeom.Cube.Define(stage,'/Collider'); hidden.AddTranslateOp().Set((0,0,-5))
        hidden.CreateVisibilityAttr('invisible'); UsdPhysics.CollisionAPI.Apply(hidden.GetPrim())
        for mode,z in [('auto',-5),('authored',-5),('visual',0)]:
            bundle=build_bundle(context(stage,collision=mode))
            self.assertAlmostEqual(bundle.collision.bounds.mean(axis=0)[2],z)
            self.assertEqual(bundle.report['counts']['collision_triangles'],12)
        self.assertIsNone(build_bundle(context(stage,collision='none')).collision)
        stage.RemovePrim('/Collider')
        bundle=build_bundle(context(stage))
        self.assertEqual(bundle.report['collision_policy'],'visual-fallback')
        with self.assertRaises(ValueError): build_bundle(context(stage,collision='authored'))
        curves=UsdGeom.BasisCurves.Define(stage,'/Collider')
        UsdPhysics.CollisionAPI.Apply(curves.GetPrim())
        with self.assertRaises(ValueError): build_bundle(context(stage))

    def test_exported_scene_maps_back_to_argos(self):
        stage=make_stage(self,axis='Y',meters_per_unit=.01)
        cube=UsdGeom.Cube.Define(stage,'/Box'); cube.AddTranslateOp().Set((100,0,0))
        bundle=build_bundle(context(stage)); original=bundle.visual.bounds.copy()
        path=Path(stage.GetRootLayer().realPath).parent/'visual.glb'
        export_glb(bundle.visual,path)
        loaded=trimesh.load(path,force='scene')
        loaded.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[1,0,0]))
        np.testing.assert_allclose(loaded.bounds,original,atol=1e-6)
        np.testing.assert_allclose(bundle.visual.bounds,original)
        np.testing.assert_allclose(original,[[.99,-.01,-.01],[1.01,.01,.01]],atol=1e-6)

    def test_unsupported_visual_geometry_is_not_a_required_collider(self):
        for mode in ('auto','visual','none'):
            stage=make_stage(self); UsdGeom.Cube.Define(stage,'/Box')
            curves=UsdGeom.BasisCurves.Define(stage,'/Curve')
            bundle=build_bundle(context(stage,collision=mode))
            self.assertEqual(bundle.report['counts']['visual_triangles'],12)
            self.assertTrue(any(d['code']=='geometry' and d['path']=='/Curve' for d in bundle.report['diagnostics']))
            UsdPhysics.CollisionAPI.Apply(curves.GetPrim())
            if mode=='auto':
                with self.assertRaisesRegex(ValueError,'collider'): build_bundle(context(stage,collision=mode))
            else: self.assertEqual(build_bundle(context(stage,collision=mode)).report['counts']['visual_triangles'],12)

    def test_selected_collider_approximation_is_not_silently_ignored(self):
        stage=make_stage(self);mesh=author_mesh(stage,'/Mesh',[[0,0,0],[1,0,0],[0,1,0]],[[0,1,2]])
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
        for mode in ('auto','authored'):
            with self.assertRaisesRegex(ValueError,'/Mesh.*approximation'): build_bundle(context(stage,collision=mode))
        for mode in ('none','visual'):
            self.assertEqual(build_bundle(context(stage,collision=mode)).report['counts']['visual_triangles'],1)

    def test_mesh_light_keeps_visual_and_authored_collision(self):
        stage=make_stage(self); UsdGeom.Cube.Define(stage,'/Box')
        mesh=author_mesh(stage,'/Emitter',[[0,0,0],[1,0,0],[0,1,0]],[[0,1,2]])
        UsdLux.MeshLightAPI.Apply(mesh.GetPrim()); UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        bundle=build_bundle(context(stage))
        self.assertEqual(bundle.report['counts']['visual_triangles'],13)
        self.assertEqual(bundle.report['counts']['collision_triangles'],1)
        self.assertTrue(any(s['path']=='/Emitter' and s['collision'] for s in bundle.report['sources']))
        self.assertTrue(any(d['code']=='light' and d['path']=='/Emitter' for d in bundle.report['diagnostics']))

    def test_shared_prototype_converts_once_and_keeps_instance_materials(self):
        stage=make_stage(self); UsdGeom.Xform.Define(stage,'/Model'); UsdGeom.Cube.Define(stage,'/Model/Box')
        for path,color in [('/A',(1,0,0)),('/B',(0,1,0))]:
            root=UsdGeom.Xform.Define(stage,path); root.GetPrim().GetReferences().AddInternalReference('/Model'); root.GetPrim().SetInstanceable(True)
            material=UsdShade.Material.Define(stage,path+'Material'); shader=UsdShade.Shader.Define(stage,path+'Material/Surface')
            shader.CreateIdAttr('UsdPreviewSurface'); shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(color)
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
            UsdShade.MaterialBindingAPI.Apply(root.GetPrim()).Bind(material)
        with mock.patch('usd_import.bundle.convert_geometry',wraps=convert_geometry) as converter:
            bundle=build_bundle(context(stage,roots=('/A','/B')))
            self.assertEqual(converter.call_count,1)
        self.assertEqual(len(bundle.visual.geometry),2);self.assertEqual(len(bundle.collision.geometry),1)
        self.assertEqual({tuple(m.visual.material.baseColorFactor[:3]) for m in bundle.visual.geometry.values()},{(255,0,0),(0,255,0)})
        self.assertEqual({s['part'] for s in bundle.report['sources']},{'/A/Box','/B/Box'})

    def test_inherited_instance_colors_are_not_cached_across_owners(self):
        stage=make_stage(self);UsdGeom.Xform.Define(stage,'/Model');UsdGeom.Cube.Define(stage,'/Model/Box')
        for path,color in [('/A',(1,0,0)),('/B',(0,1,0))]:
            root=UsdGeom.Xform.Define(stage,path);root.GetPrim().GetReferences().AddInternalReference('/Model');root.GetPrim().SetInstanceable(True)
            UsdGeom.PrimvarsAPI(root).CreatePrimvar('displayColor',Sdf.ValueTypeNames.Color3fArray,'constant').Set([color])
        bundle=build_bundle(context(stage,roots=('/A','/B')))
        self.assertEqual({tuple(m.visual.vertex_colors[0,:3]) for m in bundle.visual.geometry.values()},{(255,0,0),(0,255,0)})

    def test_shared_diagonal_vertices_are_not_duplicated(self):
        stage=make_stage(self)
        author_mesh(stage,'/Mesh',[[0,0,0],[1,0,0],[1,1,0],[0,1,0]],[[0,1,2,3]])
        bundle=build_bundle(context(stage))
        self.assertEqual(len(next(iter(bundle.visual.geometry.values())).vertices),4)
        self.assertEqual(len(next(iter(bundle.collision.geometry.values())).vertices),4)

    def test_instance_reuse_and_reflection(self):
        stage=make_stage(self)
        UsdGeom.Xform.Define(stage,'/Model'); UsdGeom.Cube.Define(stage,'/Model/Box')
        for path,x in [('/A',0),('/B',4)]:
            inst=UsdGeom.Xform.Define(stage,path)
            inst.GetPrim().GetReferences().AddInternalReference('/Model'); inst.GetPrim().SetInstanceable(True)
            inst.AddTranslateOp().Set((x,0,0))
        bundle=build_bundle(context(stage,roots=('/A','/B','/A')))
        self.assertEqual(len(bundle.visual.geometry),1)
        self.assertEqual(len(bundle.visual.graph.nodes_geometry),2)
        stage.GetPrimAtPath('/B').SetInstanceable(False)
        UsdGeom.Xform(stage.GetPrimAtPath('/B')).AddScaleOp().Set((-2,1,1))
        bundle=build_bundle(context(stage,roots=('/A','/B')))
        for node in bundle.visual.graph.nodes_geometry:
            matrix,key=bundle.visual.graph[node]
            self.assertGreater(np.linalg.det(matrix[:3,:3]),0)
            mesh=bundle.visual.geometry[key]
            self.assertTrue(np.all(np.einsum('ij,ij->i',mesh.face_normals,mesh.vertex_normals[mesh.faces].mean(axis=1))>.99))
        np.testing.assert_allclose(bundle.visual.bounds,[[-1,-1,-1],[6,1,1]])

    def test_exported_material_overrides_and_collision_untextured(self):
        stage,mesh,_,shader=fixture(self,emissiveColor=(3,0,0),opacityThreshold=.3)
        UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'vertex').Set([(0,0),(1,0),(0,1)])
        texture(stage,shader,'diffuseColor',[[[255,0,0],[0,255,0]],[[0,0,255],[255,255,255]]],colorspace='sRGB',wrap='mirror')
        texture(stage,shader,'normal',[[[128,128,255]]],scale=(2,2,2,1),bias=(-1,-1,-1,0))
        bundle=build_bundle(context(stage))
        directory=Path(stage.GetRootLayer().realPath).parent
        export_glb(bundle.visual,directory/'visual.glb'); export_glb(bundle.collision,directory/'collision.glb')
        tree=glb_json(directory/'visual.glb'); material=tree['materials'][0]
        self.assertEqual(material['alphaMode'],'MASK')
        self.assertAlmostEqual(material['alphaCutoff'],.3)
        self.assertEqual(material['extensions']['KHR_materials_emissive_strength']['emissiveStrength'],3)
        self.assertEqual(tree['samplers'][0]['wrapS'],33648)
        self.assertIn('normalTexture',material)
        self.assertTrue(all('bufferView' in image and 'uri' not in image for image in tree['images']))
        self.assertNotIn('images',glb_json(directory/'collision.glb'))

    def test_asymmetric_normal_matches_exported_uv_tangent_frame(self):
        stage,mesh,_,shader=fixture(self)
        UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'vertex').Set([(0,0),(1,0),(0,1)])
        texture(stage,shader,'normal',[[[128,204,230]]],scale=(2,2,2,1),bias=(-1,-1,-1,0))
        path=Path(stage.GetRootLayer().realPath).parent/'normal-direction.glb'
        export_glb(build_bundle(context(stage)).visual,path)
        tree=glb_json(path); raw=path.read_bytes(); binary=28+struct.unpack_from('<I',raw,12)[0]
        primitive=tree['meshes'][0]['primitives'][0]
        arrays={}
        for name,width in [('POSITION',3),('NORMAL',3),('TEXCOORD_0',2)]:
            accessor=tree['accessors'][primitive['attributes'][name]];view=tree['bufferViews'][accessor['bufferView']]
            arrays[name]=np.frombuffer(raw,dtype='<f4',count=accessor['count']*width,offset=binary+view.get('byteOffset',0)+accessor.get('byteOffset',0)).reshape(-1,width)
        slot=tree['materials'][0]['normalTexture']['index']; image=tree['images'][tree['textures'][slot]['source']]
        view=tree['bufferViews'][image['bufferView']];start=binary+view.get('byteOffset',0)
        with Image.open(io.BytesIO(raw[start:start+view['byteLength']])) as image: normal=np.asarray(image)[0,0,:3]/255*2-1
        positions=arrays['POSITION']; uv=arrays['TEXCOORD_0']
        tangent,bitangent=np.linalg.solve(uv[1:]-uv[0],positions[1:]-positions[0])
        tangent/=np.linalg.norm(tangent);bitangent/=np.linalg.norm(bitangent)
        reconstructed=tangent*normal[0]+bitangent*normal[1]+arrays['NORMAL'][0]*normal[2]
        # USD's positive tangent-Y slope remains positive after glTF's V reflection.
        self.assertGreater(reconstructed[1],.5)

    def test_display_opacity_has_blend_material_and_corner_colors(self):
        stage=make_stage(self)
        mesh=author_mesh(stage,'/Mesh',[[0,0,0],[1,0,0],[0,1,0]],[[0,1,2]])
        mesh.CreateDisplayColorPrimvar('constant').Set([(1,0,0)])
        mesh.CreateDisplayOpacityPrimvar('constant').Set([.5])
        bundle=build_bundle(context(stage)); path=Path(stage.GetRootLayer().realPath).parent/'colors.glb'
        export_glb(bundle.visual,path); tree=glb_json(path)
        self.assertEqual(tree['materials'][0]['alphaMode'],'BLEND')
        self.assertIn('COLOR_0',tree['meshes'][0]['primitives'][0]['attributes'])

    def test_deterministic_export_and_empty_scene(self):
        stage=make_stage(self); UsdGeom.Cube.Define(stage,'/Box')
        directory=Path(stage.GetRootLayer().realPath).parent
        for name in ('one','two'):
            export_glb(build_bundle(context(stage)).visual,directory/(name+'.glb'))
        self.assertEqual((directory/'one.glb').read_bytes(),(directory/'two.glb').read_bytes())
        stage.RemovePrim('/Box')
        with self.assertRaisesRegex(ValueError,'visual'): build_bundle(context(stage))

    def test_authored_normals_survive_copy_and_export(self):
        stage=make_stage(self)
        mesh=author_mesh(stage,'/Mesh',[[0,0,0],[1,0,0],[1,1,0],[0,1,0]],[[0,1,2,3]])
        mesh.CreateNormalsAttr([(1,0,1)]); mesh.SetNormalsInterpolation('constant')
        mesh.AddScaleOp().Set((2,1,1))
        bundle=build_bundle(context(stage))
        path=Path(stage.GetRootLayer().realPath).parent/'normals.glb'
        export_glb(bundle.visual,path)
        loaded=trimesh.load(path,force='scene')
        loaded.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2,[1,0,0]))
        for node in loaded.graph.nodes_geometry:
            transform,key=loaded.graph[node]
            normals=loaded.geometry[key].vertex_normals @ np.linalg.inv(transform[:3,:3])
            normals/=np.linalg.norm(normals,axis=1)[:,None]
            np.testing.assert_allclose(normals,[[1/np.sqrt(5),0,2/np.sqrt(5)]]*4,atol=1e-6)

    def test_export_size_error_is_actionable(self):
        stage=make_stage(self); UsdGeom.Cube.Define(stage,'/Box')
        scene=build_bundle(context(stage)).visual
        path=Path(stage.GetRootLayer().realPath).parent/'too-big.glb'
        with mock.patch('trimesh.exchange.gltf.export_glb',side_effect=struct.error('I too large')):
            with self.assertRaisesRegex(ValueError,'GLB'): export_glb(scene,path)
        self.assertFalse(path.exists())


if __name__=='__main__': unittest.main()
