import unittest
import numpy as np
from pxr import UsdGeom, UsdPhysics, Gf
from support import make_stage, context, author_mesh
from usd_import.scene import iter_occurrences, is_visual, is_collider, warn


class SceneTests(unittest.TestCase):
    def test_y_up_centimeters(self):
        stage = make_stage(self, axis='Y', meters_per_unit=0.01)
        UsdGeom.Cube.Define(stage, '/World/Box')
        ctx = context(stage)
        np.testing.assert_allclose(ctx.stage_to_argos @ [0,100,0,1], [0,0,1,1])
        self.assertEqual(len(list(iter_occurrences(ctx))), 1)

    def test_metadata_defaults(self):
        stage = make_stage(self, axis=None, meters_per_unit=None)
        ctx = context(stage)
        self.assertEqual(ctx.report['metadata'], {'up_axis':'Y','meters_per_unit':0.01})

    def test_reset_and_time(self):
        stage = make_stage(self, start=1004)
        parent = UsdGeom.Xform.Define(stage, '/World')
        parent.AddTranslateOp().Set((100,0,0))
        box = UsdGeom.Cube.Define(stage, '/World/Box')
        box.SetResetXformStack(True)
        op = box.AddTranslateOp()
        op.Set((1,2,3), 1004); op.Set((4,5,6), 1005)
        for options, expected in [({},[1,2,3,1]),({'time':1005},[4,5,6,1])]:
            ctx = context(stage, **options)
            np.testing.assert_allclose(next(iter_occurrences(ctx)).transform @ [0,0,0,1], expected)

    def test_roots_deduplicate_and_validate(self):
        stage = make_stage(self)
        UsdGeom.Cube.Define(stage, '/World/Box')
        ctx = context(stage, roots=('/World','/World/Box','/World'))
        self.assertEqual(len(list(iter_occurrences(ctx))), 1)
        for root in ('/Missing','World','/World.x'):
            with self.subTest(root=root), self.assertRaises(ValueError):
                context(stage, roots=(root,))

    def test_visibility_purpose_and_collision(self):
        stage = make_stage(self)
        parent = UsdGeom.Xform.Define(stage, '/World')
        UsdPhysics.CollisionAPI.Apply(parent.GetPrim())
        cube = UsdGeom.Cube.Define(stage, '/World/Box')
        cube.CreateVisibilityAttr('invisible'); cube.CreatePurposeAttr('proxy')
        ctx = context(stage); occ = next(iter_occurrences(ctx))
        self.assertFalse(is_visual(occ,ctx)); self.assertFalse(is_collider(occ,ctx))
        api = UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        ctx = context(stage); occ = next(iter_occurrences(ctx))
        self.assertTrue(is_collider(occ,ctx))
        api.CreateCollisionEnabledAttr(False)
        ctx = context(stage)
        self.assertFalse(is_collider(next(iter_occurrences(ctx)),ctx))

    def test_native_instances(self):
        stage = make_stage(self)
        UsdGeom.Xform.Define(stage, '/Model')
        UsdGeom.Cube.Define(stage, '/Model/Box')
        for path, x in [('/A',1),('/B',2)]:
            inst = UsdGeom.Xform.Define(stage,path)
            inst.GetPrim().GetReferences().AddInternalReference('/Model')
            inst.GetPrim().SetInstanceable(True)
            inst.AddTranslateOp().Set((x,0,0))
        ctx = context(stage, roots=('/A','/B'))
        occ = list(iter_occurrences(ctx))
        self.assertEqual([o.key for o in occ], ['/A/Box','/B/Box'])
        self.assertEqual([o.transform[0,3] for o in occ], [1,2])

    def test_point_instances_masks_and_prototype_transform(self):
        stage = make_stage(self)
        inst = UsdGeom.PointInstancer.Define(stage,'/Instances')
        proto = UsdGeom.Xform.Define(stage,'/Instances/Prototype')
        proto.AddTranslateOp().Set((5,0,0))
        cube = UsdGeom.Cube.Define(stage,'/Instances/Prototype/Box')
        cube.AddTranslateOp().Set((0,2,0))
        inst.CreatePrototypesRel().SetTargets(['/Instances/Prototype'])
        inst.CreateProtoIndicesAttr([0,0,0])
        inst.CreatePositionsAttr([(1,0,0),(2,0,0),(3,0,0)])
        inst.CreateIdsAttr([10,11,12]); inst.CreateInvisibleIdsAttr([11])
        ctx = context(stage); occ = list(iter_occurrences(ctx))
        self.assertEqual(len(occ),2)
        np.testing.assert_allclose([o.transform[:3,3] for o in occ], [[6,2,0],[8,2,0]])
        self.assertEqual(ctx.report['counts']['point_instances'],2)
        inst.CreateProtoIndicesAttr([0,7,0])
        with self.assertRaises(ValueError): list(iter_occurrences(context(stage)))

    def test_invalid_numeric_inputs_and_transforms(self):
        stage = make_stage(self)
        box = UsdGeom.Cube.Define(stage,'/Box')
        for options in ({'light_scale':0},{'light_range':float('nan')},{'time':float('inf')},
                        {'stops':float('nan')},{'texture_size':-1},{'name':'../bad'}):
            with self.subTest(options=options), self.assertRaises(ValueError): context(stage,**options)
        box.AddScaleOp().Set((0,1,1))
        with self.assertRaisesRegex(ValueError,'Box'): list(iter_occurrences(context(stage)))

    def test_missing_composition_asset(self):
        stage = make_stage(self)
        UsdGeom.Xform.Define(stage,'/Broken').GetPrim().GetReferences().AddReference('missing.usda')
        with self.assertRaisesRegex(ValueError,'missing.usda'): context(stage)

    def test_strict_diagnostics(self):
        ctx = context(make_stage(self), strict=True)
        with self.assertRaisesRegex(ValueError,'/Mesh'):
            warn(ctx,'subdivision','/Mesh','control cage only')
        self.assertEqual(ctx.report['diagnostics'][0]['code'],'subdivision')


if __name__ == '__main__': unittest.main()
