#!/usr/bin/env python3
"""Opt-in real-engine check of an imported bundle's rendering and floor ray."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image


def _fmt(values):
    return ','.join(format(float(v),'.9g') for v in values)


def run_smoke(config,argos,mesh_library,ray_origin,expected_distance,camera_position,camera_look_at,output):
    config=Path(config).resolve(); argos=Path(argos).resolve(); mesh_library=Path(mesh_library).resolve(); output=Path(output).resolve()
    if not argos.is_file() or not os.access(argos,os.X_OK) or not mesh_library.is_file():
        raise ValueError('provide a runnable ARGoS executable and built libmesh_jolt_loop_functions')
    if any(len(v)!=3 or not all(math.isfinite(x) for x in v) for v in (ray_origin,camera_position,camera_look_at)):
        raise ValueError('ray/camera coordinates must be three finite numbers')
    if not math.isfinite(expected_distance) or expected_distance<=0: raise ValueError('expected floor distance must be positive and finite')
    if np.linalg.norm(np.asarray(camera_position)-camera_look_at)<1e-12: raise ValueError('camera position equals look-at point')
    if output.exists() and any(output.iterdir()): raise ValueError('smoke output must be new or empty (stale frames cannot prove acceptance)')
    tree=ET.parse(config); root=tree.getroot()
    if root.find('./arena/mesh') is None: raise ValueError('smoke ray requires a collision mesh')
    for node,attr in [(n,'file') for n in root.findall('./arena/mesh')]+[(n,'model') for n in root.findall('./media/photorealism/scenery/prop')]:
        source=Path(node.get(attr)); source=source if source.is_absolute() else config.parent/source
        if not source.is_file(): raise ValueError(f'missing bundle asset: {source}')
        node.set(attr,str(source.resolve()))
    experiment=root.find('./framework/experiment'); experiment.set('length','1'); experiment.set('ticks_per_second','10')
    visual=root.find('./visualization')
    if visual is not None: root.remove(visual)
    previous=root.find('./loop_functions')
    if previous is not None: root.remove(previous)
    loops=ET.SubElement(root,'loop_functions',library=str(mesh_library),label='mesh_loop_functions',
                       ray_length=str(max(expected_distance+10,20)),ray_tolerance='0.01')
    ET.SubElement(loops,'ray',label='usd-floor',origin=_fmt(ray_origin),direction='0,0,-1',expect=format(expected_distance,'.9g'))
    medium=root.find('./media/photorealism')
    if medium is None: raise ValueError('smoke rendering requires photorealism medium')
    camera=medium.find('debug_camera')
    if camera is None: camera=ET.SubElement(medium,'debug_camera',resolution='960,540',fov='70')
    camera.set('position',_fmt(camera_position)); camera.set('look_at',_fmt(camera_look_at)); camera.set('period','1'); camera.set('dump',str(output/'frames'))
    size=tuple(map(int,camera.get('resolution','960,540').split(',')))
    output.mkdir(parents=True,exist_ok=True)
    prepared=output/'smoke.argos'; ET.indent(root,space='  '); tree.write(prepared,encoding='utf-8',xml_declaration=True)
    log=output/'argos.log'
    try:
        result=subprocess.run([str(argos),'-z','-c',str(prepared)],cwd=output,env=os.environ.copy(),
                              stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,check=True,timeout=900)
        text=result.stdout or ''; log.write_text(text)
    except (subprocess.CalledProcessError,subprocess.TimeoutExpired,OSError) as error:
        text=str(getattr(error,'stdout',None) or getattr(error,'output',None) or error)
        log.write_text(text)
        raise RuntimeError(f'ARGoS smoke failed; log {log}: {text[-4000:]}') from error
    if not re.search(r'ray checks 1, failures 0\b',text):
        raise RuntimeError(f'no successful Jolt floor-ray confirmation; log {log}: {text[-4000:]}')
    frames=sorted((output/'frames').glob('*.png'))
    if not frames: raise RuntimeError(f'ARGoS produced no frames; log {log}')
    records=[]
    for path in frames:
        with Image.open(path) as image:
            pixels=np.asarray(image.convert('RGB'))
            if image.size!=size: raise RuntimeError(f'unexpected frame dimensions: {path}: {image.size}, expected {size}')
        variance=float(pixels.astype(float).var())
        # Spatial variance, not variance between constant RGB channels.
        spatial=float(pixels.astype(float).var(axis=(0,1)).sum())
        if not pixels.any() or spatial==0: raise RuntimeError(f'all-black/constant render: {path}; log {log}')
        records.append(dict(path=str(path),width=size[0],height=size[1],variance=variance,spatial_variance=spatial))
    report=dict(config=str(prepared),original_config=str(config),log=str(log),frames=records,
                ray=dict(origin=list(ray_origin),direction=[0,0,-1],expected_distance=expected_distance,tolerance=.01,failures=0))
    (output/'smoke.json').write_text(json.dumps(report,indent=2,sort_keys=True,allow_nan=False)+'\n')
    return report


def _vector(text):
    try:
        values=tuple(map(float,text.split(',')))
        if len(values)!=3 or not all(map(math.isfinite,values)): raise ValueError()
        return values
    except ValueError as error: raise argparse.ArgumentTypeError('expected three comma-separated finite numbers') from error


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('config','argos','mesh-library','output'): parser.add_argument('--'+name,type=Path,required=True)
    for name in ('ray-origin','camera-position','camera-look-at'): parser.add_argument('--'+name,type=_vector,required=True)
    parser.add_argument('--expected-distance',type=float,required=True)
    args=parser.parse_args(argv)
    try: result=run_smoke(**vars(args))
    except (ValueError,RuntimeError,OSError) as error: parser.exit(1,str(error)+'\n')
    print(json.dumps(result,indent=2))
    return 0


if __name__=='__main__': raise SystemExit(main())
