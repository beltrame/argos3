#!/usr/bin/env python3
"""Convert a local static OpenUSD environment into a portable ARGoS bundle."""
import argparse
from pathlib import Path
import re
import sys


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',help='local .usd/.usda/.usdc composed stage (not USDZ)')
    parser.add_argument('--out',type=Path,default=Path('.'),help='output root; bundle goes in <out>/<name>/')
    parser.add_argument('--name',help='safe bundle name (default: slug of source stem)')
    parser.add_argument('--time',type=float,help='snapshot time code; default authored start or USD default')
    parser.add_argument('--root',dest='roots',action='append',default=[],help='absolute prim path; repeat to select multiple roots')
    parser.add_argument('--purpose',choices=('render','proxy'),default='render')
    parser.add_argument('--collision',choices=('auto','authored','visual','none'),default='auto')
    parser.add_argument('--texture-size',type=int,default=1024,help='longest texture side in pixels; 0 retains resolution')
    parser.add_argument('--light-scale',type=float,default=1,help='positive USD-to-Filament intensity calibration')
    parser.add_argument('--light-range',type=float,default=20,help='local-light falloff in meters')
    parser.add_argument('--stops',type=float,default=6,help='exposure stops above sunny 16')
    parser.add_argument('--strict',action='store_true',help='make conversion warnings fatal')
    parser.add_argument('--overwrite',action='store_true',help='replace an existing importer-generated bundle safely')
    args=parser.parse_args(argv)
    try:
        from usd_import.scene import Options, read_stage
        from usd_import.bundle import build_bundle, write_bundle, _destination
    except ImportError as error:
        print(f'Missing importer dependency ({error}). Install with: python -m pip install -r {Path(__file__).with_name("requirements-usd.txt")}',file=sys.stderr)
        return 1
    name=args.name if args.name is not None else re.sub('[^a-z0-9]+','_',Path(args.source).stem.lower()).strip('_') or 'scene'
    try:
        if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]*',name): raise ValueError('invalid bundle name')
        _destination(args.out,name,args.overwrite)
        settings=vars(args).copy(); settings.update(source=Path(args.source),name=name,roots=tuple(args.roots))
        ctx=read_stage(Options(**settings)); bundle=build_bundle(ctx); destination=write_bundle(bundle,ctx)
    except Exception as error:
        print(f'USD import failed: {error}',file=sys.stderr)
        return 1
    for diagnostic in ctx.report['diagnostics']:
        print(f'warning: {diagnostic["path"]}: {diagnostic["message"]} ({diagnostic["code"]})',file=sys.stderr)
    print(destination)
    return 0


if __name__=='__main__': sys.exit(main())
