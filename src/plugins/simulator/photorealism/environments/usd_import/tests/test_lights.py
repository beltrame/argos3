import unittest
import numpy as np
from pxr import UsdGeom, UsdLux
from support import make_stage, context
from usd_import.scene import iter_occurrences
from usd_import.lights import convert_lights


class LightTests(unittest.TestCase):
    def convert(self,stage,**opts):
        ctx=context(stage,**opts)
        return convert_lights(ctx,list(iter_occurrences(ctx))),ctx

    def test_intensity_exposure_and_scale(self):
        stage=make_stage(self)
        lamp=UsdLux.SphereLight.Define(stage,'/Lamp')
        lamp.CreateIntensityAttr(10); lamp.CreateExposureAttr(2)
        lighting,_=self.convert(stage,light_scale=3)
        self.assertEqual(lighting.local[0]['intensity'],120)
        self.assertEqual(lighting.local[0]['falloff'],20)

    def test_frame_spot_and_temperature(self):
        stage=make_stage(self,axis='Y',meters_per_unit=.01)
        lamp=UsdLux.SphereLight.Define(stage,'/Lamp')
        lamp.AddTranslateOp().Set((100,200,300))
        lamp.CreateEnableColorTemperatureAttr(True); lamp.CreateColorTemperatureAttr(6500)
        shaping=UsdLux.ShapingAPI.Apply(lamp.GetPrim())
        shaping.CreateShapingConeAngleAttr(30); shaping.CreateShapingConeSoftnessAttr(0)
        light=self.convert(stage)[0].local[0]
        np.testing.assert_allclose(light['position'],[1,-3,2])
        np.testing.assert_allclose(light['direction'],[0,1,0])
        self.assertEqual(light['type'],'spot'); self.assertEqual(light['outer_angle'],30)
        self.assertEqual(light['inner_angle'],30)
        np.testing.assert_allclose(light['color'],[1,1,1],atol=.06)
        lamp.CreateColorTemperatureAttr(2500)
        warm=self.convert(stage)[0].local[0]['color']
        self.assertGreater(warm[0],warm[2])

    def test_area_and_strongest_distant(self):
        stage=make_stage(self)
        for schema,path in [(UsdLux.RectLight,'/Rect'),(UsdLux.DiskLight,'/Disk'),(UsdLux.CylinderLight,'/Cylinder')]:
            schema.Define(stage,path)
        for path,intensity in [('/A',10),('/B',20),('/C',20)]:
            UsdLux.DistantLight.Define(stage,path).CreateIntensityAttr(intensity)
        lighting,ctx=self.convert(stage)
        self.assertEqual(len(lighting.local),3)
        self.assertEqual(lighting.sun['source'],'/B')
        self.assertTrue(any(d['code']=='area_light' for d in ctx.report['diagnostics']))
        self.assertTrue(any(d['code']=='distant_light' for d in ctx.report['diagnostics']))
        with self.assertRaises(ValueError): self.convert(stage,strict=True)

    def test_unusable_lights_add_inspectable_defaults(self):
        stage=make_stage(self)
        UsdLux.DomeLight.Define(stage,'/Dome')
        UsdLux.SphereLight.Define(stage,'/Zero').CreateIntensityAttr(0)
        hidden=UsdLux.SphereLight.Define(stage,'/Hidden'); hidden.CreateVisibilityAttr('invisible')
        lighting,ctx=self.convert(stage)
        self.assertTrue(lighting.defaulted)
        self.assertEqual(lighting.sun['intensity'],100000)
        self.assertEqual(lighting.local,[])
        self.assertTrue(any(d['code']=='dome_light' for d in ctx.report['diagnostics']))

    def test_invalid_intensities_and_exposure(self):
        for attr,value in [('intensity',float('nan')),('intensity',-1),('exposure',10000)]:
            stage=make_stage(self); lamp=UsdLux.SphereLight.Define(stage,'/Lamp')
            lamp.GetPrim().GetAttribute('inputs:'+attr).Set(value)
            with self.subTest(attr=attr,value=value),self.assertRaisesRegex(ValueError,'Lamp'): self.convert(stage)

    def test_linking_and_softness_diagnostics(self):
        stage=make_stage(self); lamp=UsdLux.SphereLight.Define(stage,'/Lamp')
        lamp.GetPrim().CreateRelationship('filters').SetTargets(['/Filter'])
        UsdLux.ShapingAPI.Apply(lamp.GetPrim()).CreateShapingConeSoftnessAttr(.5)
        _,ctx=self.convert(stage)
        self.assertTrue(any(d['code']=='light_linking' for d in ctx.report['diagnostics']))
        self.assertTrue(any(d['code']=='shaping' for d in ctx.report['diagnostics']))


if __name__=='__main__': unittest.main()
