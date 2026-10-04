import unittest
from unittest import mock
import numpy as np
from pxr import UsdGeom, UsdPhysics, Sdf
from support import make_stage, context, author_mesh
from usd_import.scene import iter_occurrences
from usd_import.geometry import convert_geometry


class GeometryTests(unittest.TestCase):
    def convert(self, stage, **opts):
        ctx = context(stage, **opts)
        return convert_geometry(next(iter_occurrences(ctx)),ctx), ctx

    def test_concave_face_area(self):
        s = make_stage(self)
        author_mesh(s,'/Mesh',[[0,0,0],[2,0,0],[2,2,0],[1,1,0],[0,2,0]],[[0,1,2,3,4]])
        parts,_ = self.convert(s)
        self.assertEqual(sum(len(p.mesh.faces) for p in parts),3)
        self.assertAlmostEqual(sum(p.mesh.area for p in parts),3)

    def test_nonplanar_face_valid_alternate_projection(self):
        s=make_stage(self)
        author_mesh(s,'/Mesh',[[0,0,0],[.01,-.003,-.023],[.63,-.033,.295],[.634,-.029,.276]],[[0,1,2,3]])
        parts,_=self.convert(s)
        self.assertEqual(len(parts[0].mesh.faces),2)
        self.assertTrue(np.isfinite(parts[0].mesh.vertices).all())

    def test_nearly_planar_fold_is_not_a_planar_self_intersection(self):
        s=make_stage(self)
        author_mesh(s,'/Mesh',[[0,0,0],[1e-6,-.007,.792],[1e-6,-.008,.7925],[0,.312,.562]],[[0,1,2,3]])
        parts,_=self.convert(s)
        self.assertEqual(len(parts[0].mesh.faces),2)

    def test_adjacent_duplicate_positions_drop_only_degenerate_triangle(self):
        s=make_stage(self)
        mesh=author_mesh(s,'/Mesh',[[0,0,0],[1,0,0],[1,0,0],[0,1,0]],[[0,1,2,3]])
        UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'faceVarying').Set([(0,0),(1,0),(.9,.1),(0,1)])
        parts,ctx=self.convert(s)
        self.assertEqual(len(parts[0].mesh.faces),1)
        self.assertTrue(any(d['code']=='degenerate' and d['details']['count']==1 for d in ctx.report['diagnostics']))
        np.testing.assert_allclose(parts[0].uvs['st'],[[0,0],[1,0],[0,1]])

    def test_ambiguous_subdivision_quad_is_diagnosed(self):
        s=make_stage(self)
        mesh=author_mesh(s,'/Mesh',[[0,0,0],[1,1,0],[0,1,0],[1,0,0]],[[0,1,2,3]])
        with self.assertRaisesRegex(ValueError,'self-intersecting'): self.convert(s)
        mesh.CreateSubdivisionSchemeAttr('catmullClark')
        parts,ctx=self.convert(s)
        self.assertEqual(len(parts[0].mesh.faces),2)
        self.assertTrue(any(d['code']=='ambiguous_cage_quad' for d in ctx.report['diagnostics']))
        with self.assertRaises(ValueError): self.convert(s,strict=True)

    def test_planar_boundary_touch_is_rejected(self):
        s=make_stage(self)
        author_mesh(s,'/Mesh',[[0,0,0],[2,0,0],[2,2,0],[1,0,0],[0,2,0]],[[0,1,2,3,4]])
        with self.assertRaisesRegex(ValueError,'self-intersecting'): self.convert(s)

    def test_analytic_left_handed_orientation(self):
        s=make_stage(self)
        cube=UsdGeom.Cube.Define(s,'/Cube')
        right,_=self.convert(s)
        cube.CreateOrientationAttr('leftHanded')
        left,_=self.convert(s)
        np.testing.assert_allclose(left[0].mesh.face_normals,-right[0].mesh.face_normals)

    def test_folded_quad_uses_3d_diagonal_with_diagnostic(self):
        s=make_stage(self)
        author_mesh(s,'/Mesh',[[0,0,0],[0,.3285,-.3823],[-1e-6,.0044,-.6223],[-1e-6,.0054,-.6227]],[[0,1,2,3]])
        parts,ctx=self.convert(s)
        self.assertEqual(len(parts[0].mesh.faces),2)
        self.assertTrue(any(d['code']=='twisted_quad' for d in ctx.report['diagnostics']))
        with self.assertRaises(ValueError): self.convert(s,strict=True)

    def test_holes_orientation_and_degenerates(self):
        s = make_stage(self)
        m=author_mesh(s,'/Mesh',[[0,0,0],[1,0,0],[0,1,0],[0,0,0]],[[0,1,2],[0,1,2],[0,3,3]])
        m.CreateHoleIndicesAttr([1]); m.CreateOrientationAttr('leftHanded')
        parts,ctx=self.convert(s)
        self.assertEqual(len(parts[0].mesh.faces),1)
        np.testing.assert_allclose(parts[0].mesh.face_normals,[[0,0,-1]])
        self.assertTrue(any(d['code']=='degenerate' for d in ctx.report['diagnostics']))

    def test_face_corner_attributes_and_subsets(self):
        s = make_stage(self)
        m=author_mesh(s,'/Mesh',[[0,0,0],[1,0,0],[1,1,0],[0,1,0]],[[0,1,2],[0,2,3]])
        uv=UsdGeom.PrimvarsAPI(m).CreatePrimvar('perfuv',Sdf.ValueTypeNames.TexCoord2fArray,'faceVarying')
        uv.Set([(0,0),(1,0),(1,1),(0.2,0.2),(0,1)])
        uv.SetIndices([0,1,2,3,2,4])
        m.CreateNormalsAttr([(0,0,1),(0,0,-1)]); m.SetNormalsInterpolation('uniform')
        UsdGeom.Subset.CreateGeomSubset(m,'part','face',[0],'materialBind')
        parts,_ = self.convert(s)
        self.assertEqual(len(parts),2)
        self.assertEqual(sum(len(p.mesh.faces) for p in parts),2)
        selected=next(p for p in parts if p.binding_prim.GetName()=='part')
        np.testing.assert_allclose(selected.uvs['perfuv'],[[0,0],[1,0],[1,1]])
        remainder=next(p for p in parts if p.binding_prim.GetName()=='Mesh')
        np.testing.assert_allclose(remainder.uvs['perfuv'][0],[0.2,0.2],atol=1e-7)
        np.testing.assert_allclose(remainder.mesh.vertex_normals,[[0,0,-1]]*3)

    def test_inherited_display_and_indexed_normals(self):
        s=make_stage(self)
        root=UsdGeom.Xform.Define(s,'/World')
        color=UsdGeom.PrimvarsAPI(root).CreatePrimvar('displayColor',Sdf.ValueTypeNames.Color3fArray,'constant')
        color.Set([(1,0,0)])
        m=author_mesh(s,'/World/Mesh',[[0,0,0],[1,0,0],[0,1,0]],[[0,1,2]])
        normal=UsdGeom.PrimvarsAPI(m).CreatePrimvar('normals',Sdf.ValueTypeNames.Normal3fArray,'vertex')
        normal.Set([(0,0,1)]); normal.SetIndices([0,0,0])
        parts,_=self.convert(s)
        np.testing.assert_array_equal(parts[0].mesh.visual.vertex_colors,[[255,0,0,255]]*3)
        np.testing.assert_allclose(parts[0].mesh.vertex_normals,[[0,0,1]]*3)

    def test_scalar_custom_and_constant_vector_primvars(self):
        s=make_stage(self); m=author_mesh(s,'/Mesh',[[0,0,0],[1,0,0],[0,1,0]],[[0,1,2]])
        api=UsdGeom.PrimvarsAPI(m)
        api.CreatePrimvar('productionMask',Sdf.ValueTypeNames.Int,'constant').Set(7)
        api.CreatePrimvar('perfuv',Sdf.ValueTypeNames.TexCoord2f,'constant').Set((.2,.3))
        parts,_=self.convert(s)
        np.testing.assert_allclose(parts[0].uvs['perfuv'],[[.2,.3]]*3,atol=1e-6)

    def test_analytic_dimensions(self):
        for schema in (UsdGeom.Cube,UsdGeom.Sphere,UsdGeom.Cylinder,UsdGeom.Cone,UsdGeom.Capsule):
            with self.subTest(schema=schema):
                s=make_stage(self); shape=schema.Define(s,'/Shape')
                if schema==UsdGeom.Cube:
                    shape.CreateSizeAttr(2)
                else:
                    shape.CreateRadiusAttr(1)
                    if schema!=UsdGeom.Sphere: shape.CreateHeightAttr(2)
                parts,_=self.convert(s)
                self.assertGreater(len(parts[0].mesh.faces),0)
                expected_z=4 if schema==UsdGeom.Capsule else 2
                np.testing.assert_allclose(parts[0].mesh.extents,[2,2,expected_z],atol=0.06)

    def test_invalid_indices_subsets_and_self_intersection(self):
        for faces in ([[0,1,8]],[[0,2,1,3]]):
            s=make_stage(self)
            author_mesh(s,'/Mesh',[[0,0,0],[1,0,0],[1,1,0],[0,1,0]],faces)
            with self.assertRaisesRegex(ValueError,'Mesh'): self.convert(s)
        s=make_stage(self); m=author_mesh(s,'/Mesh',[[0,0,0],[1,0,0],[0,1,0]],[[0,1,2]])
        UsdGeom.Subset.CreateGeomSubset(m,'a','face',[0],'materialBind')
        UsdGeom.Subset.CreateGeomSubset(m,'b','face',[0],'materialBind')
        with self.assertRaisesRegex(ValueError,'subset'): self.convert(s)

    def test_bad_normals_regenerated_but_bad_uv_diagnosed(self):
        s=make_stage(self); m=author_mesh(s,'/Mesh',[[0,0,0],[1,0,0],[0,1,0]],[[0,1,2]])
        m.CreateNormalsAttr([(0,0,0)]); m.SetNormalsInterpolation('constant')
        pv=UsdGeom.PrimvarsAPI(m).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'vertex')
        pv.Set([(0,0)])
        parts,ctx=self.convert(s)
        np.testing.assert_allclose(parts[0].mesh.vertex_normals,[[0,0,1]]*3)
        self.assertTrue(ctx.report['diagnostics'])
        with self.assertRaises(ValueError): self.convert(s,strict=True)

    def test_generated_normals_do_not_require_scipy(self):
        s=make_stage(self)
        author_mesh(s,'/Mesh',[[0,0,0],[1,0,0],[0,1,0]],[[0,1,2]])
        with mock.patch('trimesh.geometry.weighted_vertex_normals',side_effect=AssertionError('optional scipy path')):
            parts,_=self.convert(s)
            np.testing.assert_allclose(parts[0].mesh.vertex_normals,[[0,0,1]]*3)

    def test_subdivision_and_unsupported_collider(self):
        s=make_stage(self); m=author_mesh(s,'/Mesh',[[0,0,0],[1,0,0],[0,1,0]],[[0,1,2]])
        m.CreateSubdivisionSchemeAttr('catmullClark')
        _,ctx=self.convert(s)
        self.assertEqual(ctx.report['diagnostics'][0]['code'],'subdivision')
        with self.assertRaises(ValueError): self.convert(s,strict=True)
        s=make_stage(self); curves=UsdGeom.BasisCurves.Define(s,'/Curve')
        parts,ctx=self.convert(s)
        self.assertEqual(parts,[])
        UsdPhysics.CollisionAPI.Apply(curves.GetPrim())
        with self.assertRaises(ValueError): self.convert(s)


if __name__=='__main__': unittest.main()
