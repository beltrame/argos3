import json
import os
import subprocess
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock
import numpy as np
from PIL import Image
from pxr import UsdGeom
from support import make_stage, context
from usd_import.bundle import build_bundle, write_bundle
from smoke_argos import run_smoke


def make_smoke_config(test):
    stage=make_stage(test)
    floor=UsdGeom.Cube.Define(stage,'/Floor'); floor.AddTranslateOp().Set((0,0,-.5)); floor.AddScaleOp().Set((3,3,.5))
    wall=UsdGeom.Cube.Define(stage,'/Wall'); wall.AddTranslateOp().Set((2,0,1)); wall.AddScaleOp().Set((.1,3,1))
    root=Path(stage.GetRootLayer().realPath).parent
    ctx=context(stage,out=root/'bundle',name='scene')
    return write_bundle(build_bundle(ctx),ctx)/'scene.argos'


class SmokeTests(unittest.TestCase):
    def setup_config(self):
        config=make_smoke_config(self)
        argos=config.parent/'argos'; argos.write_text('stub'); argos.chmod(0o755)
        library=config.parent/'mesh-loops.so'; library.write_text('stub')
        return config,argos,library

    def test_failed_argos_is_not_a_pass(self):
        config,argos,library=self.setup_config()
        with mock.patch('smoke_argos.subprocess.run',side_effect=subprocess.CalledProcessError(1,['argos'],output='engine failed')) as execute:
            with self.assertRaisesRegex(RuntimeError,'engine failed'):
                run_smoke(config,argos,library,(0,0,1),1,(1,1,1),(0,0,0),config.parent/'smoke-output')
            execute.assert_called_once()

    def test_prepared_config_success_and_original_unchanged(self):
        config,argos,library=self.setup_config(); original=config.read_bytes(); output=config.parent/'smoke-output'
        def execute(command,**kwargs):
            self.assertEqual(command[1:3],['-z','-c'])
            self.assertEqual(kwargs['env'].get('ARGOS_PLUGIN_PATH'),os.environ.get('ARGOS_PLUGIN_PATH'))
            tree=ET.parse(command[3])
            self.assertEqual(tree.find('./framework/experiment').get('length'),'1')
            self.assertIsNone(tree.find('./visualization'))
            self.assertEqual(tree.find('./loop_functions').get('ray_tolerance'),'0.01')
            self.assertEqual(tree.find('./loop_functions/ray').get('expect'),'1')
            for node,attr in (('./arena/mesh','file'),('./media/photorealism/scenery/prop','model')):
                self.assertTrue(Path(tree.find(node).get(attr)).is_absolute())
            camera=tree.find('./media/photorealism/debug_camera')
            self.assertEqual(camera.get('position'),'1,1,1'); self.assertEqual(camera.get('period'),'1')
            frames=Path(camera.get('dump')); frames.mkdir()
            pixels=np.zeros((540,960,3),dtype=np.uint8); pixels[:,:480]=[255,128,0]
            Image.fromarray(pixels).save(frames/'frame_1.png')
            return subprocess.CompletedProcess(command,0,stdout='[mesh] ray checks 1, failures 0, worst error 0 m')
        with mock.patch('smoke_argos.subprocess.run',side_effect=execute):
            result=run_smoke(config,argos,library,(0,0,1),1,(1,1,1),(0,0,0),output)
        self.assertEqual(config.read_bytes(),original)
        self.assertEqual(result['ray']['failures'],0); self.assertEqual(len(result['frames']),1)
        self.assertTrue((output/'smoke.json').is_file()); json.dumps(result,allow_nan=False)

    def test_missing_constant_black_and_bad_size_frames_fail(self):
        for mode in ('missing','black','constant','size'):
            config,argos,library=self.setup_config(); output=config.parent/'smoke-output'
            def execute(command,**kwargs):
                frames=output/'frames'; frames.mkdir()
                if mode!='missing': Image.new('RGB',(10,10) if mode=='size' else (960,540),0 if mode=='black' else 128).save(frames/'frame.png')
                return subprocess.CompletedProcess(command,0,stdout='[mesh] ray checks 1, failures 0')
            with self.subTest(mode=mode),mock.patch('smoke_argos.subprocess.run',side_effect=execute):
                with self.assertRaises(RuntimeError): run_smoke(config,argos,library,(0,0,1),1,(1,1,1),(0,0,0),output)

    def test_missing_ray_confirmation_is_not_a_pass(self):
        config,argos,library=self.setup_config()
        with mock.patch('smoke_argos.subprocess.run',return_value=subprocess.CompletedProcess([],0,stdout='no loop plugin ran')):
            with self.assertRaisesRegex(RuntimeError,'ray'): run_smoke(config,argos,library,(0,0,1),1,(1,1,1),(0,0,0),config.parent/'smoke-output')


if __name__=='__main__': unittest.main()
