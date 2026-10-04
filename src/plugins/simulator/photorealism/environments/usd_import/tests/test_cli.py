import contextlib
import io
import json
import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest import mock
from pxr import UsdGeom
from support import make_stage, context
from usd_import.bundle import build_bundle, write_bundle
from import_usd_world import main


class CliTests(unittest.TestCase):
    def setup_scene(self):
        stage=make_stage(self); UsdGeom.Cube.Define(stage,'/Box'); stage.GetRootLayer().Save()
        source=Path(stage.GetRootLayer().realPath); out=source.parent/'bundles'
        return stage,source,out

    def test_generated_configuration_uses_existing_plugins(self):
        _,source,out=self.setup_scene()
        with contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(main([str(source),'--out',str(out),'--name','scene']),0)
        directory=out/'scene'; tree=ET.parse(directory/'scene.argos')
        self.assertEqual(tree.find('./media/photorealism').get('draw_floor'),'false')
        self.assertEqual(tree.find('./media/photorealism').get('backend'),'vulkan')
        self.assertEqual(tree.find('./arena/mesh').get('y_up'),'true')
        self.assertEqual(tree.find('./arena/mesh').get('double_sided'),'true')
        self.assertEqual(tree.find('./media/photorealism/scenery/prop').get('orientation'),'0,0,90')
        self.assertIsNone(tree.find('./physics_engines/jolt/floor'))
        self.assertEqual(tree.find('./physics_engines/jolt/gravity').get('g'),'9.81')
        self.assertEqual(tree.find('./arena').get('size'),'6,6,6')
        exposure=tree.find('./media/photorealism/exposure')
        self.assertEqual(float(exposure.get('aperture')),2)
        self.assertEqual(float(exposure.get('shutter_speed')),.01)
        self.assertEqual(float(exposure.get('sensitivity')),100)
        for name in ('scene.glb','scene.collision.glb','scene.lights.xml','scene.json'): self.assertTrue((directory/name).is_file())
        report=json.loads((directory/'scene.json').read_text())
        self.assertEqual(report['camera']['debug']['resolution'],'960,540')
        self.assertEqual(report['importer'],'openusd-static')
        self.assertNotIn('NaN',(directory/'scene.json').read_text())

    def test_subprocess_portable_none_and_help_without_sdk(self):
        _,source,out=self.setup_scene(); script=Path(__file__).parents[2]/'import_usd_world.py'
        result=subprocess.run([sys.executable,str(script),str(source),'--out',str(out),'--name','scene','--collision','none'],cwd='/tmp',capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)
        tree=ET.parse(out/'scene/scene.argos'); self.assertIsNone(tree.find('./arena/mesh'))
        self.assertFalse((out/'scene/scene.collision.glb').exists())
        # -S deliberately hides site-packages/SDK without changing installation.
        help_result=subprocess.run([sys.executable,'-S',str(script),'--help'],capture_output=True,text=True)
        self.assertEqual(help_result.returncode,0,help_result.stderr)
        result=subprocess.run([sys.executable,'-S',str(script),str(source)],capture_output=True,text=True)
        self.assertEqual(result.returncode,1); self.assertIn('requirements-usd.txt',result.stderr)
        self.assertNotIn('Traceback',result.stderr)

    def test_invalid_cli_and_early_existing_guard(self):
        _,source,out=self.setup_scene()
        for args in (['--name','../outside'],['--name',''],['--texture-size','-1'],['--light-scale','nan'],['--light-range','0'],['--time','inf'],['--root','Missing']):
            with contextlib.redirect_stderr(io.StringIO()),self.subTest(args=args):
                self.assertNotEqual(main([str(source),'--out',str(out)]+args),0)
        with contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(main([str(source),'--out',str(out),'--name','scene']),0)
            with mock.patch('usd_import.scene.read_stage',side_effect=AssertionError('too late')):
                self.assertEqual(main([str(source),'--out',str(out),'--name','scene']),1)
        self.assertFalse((out.parent/'outside').exists())

    def test_failed_overwrite_keeps_old_bundle(self):
        stage,source,out=self.setup_scene()
        ctx=context(stage,out=out,name='scene'); bundle=build_bundle(ctx)
        destination=write_bundle(bundle,ctx); sentinel=destination/'sentinel'; sentinel.write_text('keep')
        ctx.options.overwrite=True
        original=Path.rename
        def failure(which):
            def rename(path,target):
                if (which=='old' and path==destination) or (which=='new' and str(path.name).startswith('.scene.tmp-')):
                    raise OSError('injected '+which+' rename failure')
                return original(path,target)
            return rename
        for which in ('old','new'):
            with self.subTest(which=which),mock.patch.object(Path,'rename',failure(which)):
                with self.assertRaisesRegex(OSError,'injected'): write_bundle(bundle,ctx)
            self.assertEqual(sentinel.read_text(),'keep')
            self.assertEqual(list(out.glob('.scene.tmp-*')),[])
        with mock.patch('usd_import.bundle.export_glb',side_effect=ValueError('export failed')):
            with self.assertRaises(ValueError): write_bundle(bundle,ctx)
        self.assertEqual(sentinel.read_text(),'keep')

    def test_cleanup_failure_and_symlink_protection(self):
        stage,source,out=self.setup_scene(); ctx=context(stage,out=out,name='scene')
        bundle=build_bundle(ctx); destination=write_bundle(bundle,ctx); ctx.options.overwrite=True
        with mock.patch('shutil.rmtree',side_effect=OSError('cleanup denied')),contextlib.redirect_stderr(io.StringIO()) as errors:
            write_bundle(bundle,ctx)
        self.assertIn('backup',errors.getvalue()); self.assertTrue((destination/'scene.glb').is_file())
        self.assertEqual(len(list(out.glob('.scene.backup-*'))),1)
        ctx.options.name='link'; ctx.name='link'
        link=out/'link'; link.symlink_to(destination,target_is_directory=True)
        with self.assertRaisesRegex(ValueError,'symlink'): write_bundle(bundle,ctx)
        self.assertTrue((destination/'scene.glb').is_file())

    def test_strict_failure_leaves_no_bundle(self):
        _,source,out=self.setup_scene()
        with contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(main([str(source),'--out',str(out),'--strict']),1)
        self.assertFalse(out.exists())


if __name__=='__main__': unittest.main()
