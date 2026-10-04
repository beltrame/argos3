# Netflix Animation Studios ALab static environment target

Acceptance status and measured results: [acceptance.md](acceptance.md).
Asset copyright, license and conversion changes: [NOTICE.md](NOTICE.md).
The full selected scene now passes headless rendering, a source-derived Jolt
floor ray and a supported foot-bot pose. Interactive viewer inspection remains
unvalidated without a display. A tiny fixture alone is not this evidence.

## Download and compose

This opt-in download is large: **2,016,093,878 bytes** for the v2.3.0
structure (including texture cards) and **9,609,335,555 bytes** for
Techvar Assets v2.2.0. Extracted originals total approximately 12.6 GB;
merging duplicates about 10.4 GB before generated assets. Keep ample disk
space and RAM; do not download from normal tests.

From the photorealism directory, with the importer venv installed:

```sh
E="$PWD/environments"
PY=/tmp/usd-venv/bin/python
mkdir -p "$E/alab/downloads"
curl -L --fail -o "$E/alab/downloads/ALab-v2.3.0.zip" \
  https://github.com/DigitalProductionExampleLibrary/ALab/archive/refs/tags/v2.3.0.zip
curl -L --fail -C - -o "$E/alab/downloads/alab-techvars.v2.2.0.zip" \
  https://dpel-assets.aswf.io/usd-alab/alab-techvars.v2.2.0.zip
sha256sum "$E/alab/downloads/"*.zip
```

GitHub's structure archive endpoint did not support byte-range resume;
restart that download if interrupted. The Techvar endpoint supports it.
Hashes measured for this run:

| Archive | SHA-256 |
| --- | --- |
| ALab-v2.3.0.zip | `56d47a3cc85a687092e28e84548d04079e03640f7ceb525e252fdadaea22dc4f` |
| alab-techvars.v2.2.0.zip | `d142891ed4ad2365f8dd6b583e9dac88982131d7709dd8e6f6bc0f1516007ec6` |

Inspect listings and reject unsafe member paths before extraction:

```sh
"$PY" - "$E/alab" <<'PY'
from pathlib import Path, PurePosixPath
import stat, sys, zipfile
root = Path(sys.argv[1])
for archive in sorted((root / 'downloads').glob('*.zip')):
    with zipfile.ZipFile(archive) as z:
        members = z.infolist()
        print(archive.name, len(members), 'members',
              sum(i.file_size for i in members), 'expanded bytes')
        print('\n'.join(i.filename for i in members[:30]))
        for i in members:
            p = PurePosixPath(i.filename)
            if p.is_absolute() or '..' in p.parts or '\\' in i.filename or stat.S_ISLNK(i.external_attr >> 16):
                raise ValueError('unsafe member: ' + i.filename)
        z.extractall(root / 'source')
PY
cp -a "$E/alab/source/techvar_assets/fragment/." \
      "$E/alab/source/ALab-2.3.0/ALab/fragment/"
ALAB_SOURCE="$E/alab/source/ALab-2.3.0/ALab"
```

This merges Techvar `fragment/` into the structure's `ALab/fragment/`,
as instructed by upstream. Do not install optional Baked Procedurals,
4K EXR Texture Pack, Trailer Cameras or Splat Pack for this target.
Preserve `source/ALab-2.3.0/LICENSE.md` and upstream documentation.

The merged stage was verified to have no composition errors, frame range
1004–1057, Y-up centimeters, and both roots below. It contains production
and preview bindings; use composed Preview Surface/JPEG preview textures.

## Convert

```sh
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PXR_WORK_THREAD_LIMIT=1 \
"$PY" "$E/import_usd_world.py" "$ALAB_SOURCE/entry.usda" \
  --name alab --out "$E/alab/generated" --time 1004 \
  --root /root/alab_set01 --root /root/lightrig01 \
  --purpose render --collision auto --texture-size 1024
cp "$E/alab/NOTICE.md" "$ALAB_SOURCE/../LICENSE.md" "$E/alab/generated/alab/"
```

Do not broaden roots to include `/root/remi` or `/root/stoat` (characters).
Authored USD cameras and skydome are outside the selected roots.
`--purpose proxy` is an explicit lower-detail alternative, not a silent
fallback for failed render-purpose conversion. Retain JSON diagnostics.
Subdivision cages (including diagnosed shorter-diagonal approximations
of folded or self-intersecting cage quads), production-light approximations,
unsupported Points and dome textures are outside the initial fidelity contract. Baked preview
images may be embedded as PNG after channel/color-space conversion.
Use `--light-scale` and `--stops` to calibrate exposure rather than edit
source assets. `--strict` is intentionally unsuitable for this production
scene's documented approximations.

The generated `.argos` is robot-free, uses photorealism medium `pr`, has
no synthetic floor, and contains static Jolt collision. Normal experiment
length is zero (interactive). Only the smoke runner makes a finite test
copy; its original bundle is unchanged.

## Engine validation

Build ARGoS with `ARGOS_BUILD_JOLT=ON` and a valid `FILAMENT_DIR`; the
simulator build already enables its existing test libraries. Source the
build's `setup_env.sh` to populate `ARGOS_PLUGIN_PATH`/`LD_LIBRARY_PATH`.
On a CPU-only host, export
`VK_DRIVER_FILES=/usr/share/vulkan/icd.d/lvp_icd.json`.

The optional runner is `usd_import/tests/smoke_argos.py`; its `--help`
lists the executable/library, ray, camera and output arguments. Choose a
**source-verified** interior floor and clear supported starting pose from
[acceptance.md](acceptance.md), not the center of scene bounds. Require
nonconstant PNGs and the Jolt floor ray within 0.01 meters. A fresh output
directory is required so stale frames cannot produce a false pass.

The default exposure is dark indoors. Regenerate with **`--stops 12`** for
this target, or prepare a runtime-only interior copy without repeating mesh
conversion (equivalent exposure settings):

```sh
"$PY" - "$E/alab/generated/alab" <<'PY'
import sys, xml.etree.ElementTree as ET
from pathlib import Path
bundle = Path(sys.argv[1])
tree = ET.parse(bundle / 'alab.argos')
medium = tree.getroot().find('./media/photorealism')
medium.find('exposure').attrib.update(aperture='2', shutter_speed='0.04', sensitivity='1600')
pose = dict(position='0.494817421,-0.975042665,1.094701773',
            look_at='1.616075863,1.792697795,1.381742281')
medium.find('debug_camera').attrib.update(pose)
tree.getroot().find('./visualization/filament').attrib.update(pose)
ET.indent(tree.getroot(), space='  ')
tree.write(bundle / 'alab-interior.argos', encoding='utf-8', xml_declaration=True)
PY
argos3 -c "$E/alab/generated/alab/alab-interior.argos"
```

A graphical display is required for that interactive command. The recorded
headless interior run, floor ray and validation-only robot configuration are
listed in [acceptance.md](acceptance.md). Generated experiments remain robot-free.
