# ALab acceptance — headless scene validated; interactive viewer unavailable

Target: ALab structure v2.3.0 plus Techvar assets v2.2.0, frame **1004**,
render/default purpose, `/root/alab_set01` and `/root/lightrig01`.
Downloads and local engine build were explicitly authorized. This is a
full selected-scene conversion, not a tiny fixture or proxy substitution.

- [x] Record versioned URLs, sizes, SHA-256 hashes and notices in README/NOTICE.
- [x] Inspect archive paths, extract safely, verify merged package layout.
- [x] Verify frame range 1004–1057, Y-up centimeters and zero composition errors.
- [x] Confirm composed Preview Surface bindings and portable embedded preview textures.
- [x] Complete full conversion, recording versions, runtime/memory and diagnostics.
- [x] Independently reopen both GLBs; verify finite vertices, bounds, triangles and material/texture portability.
- [x] Confirm every exported occurrence record stays within selected roots; no Remi/Stoat characters.
- [x] Inspect embedded textures/UV attribute requirements and diagnose six omitted Points prims.
- [x] Inspect an actual source-camera interior PNG for upright/coherent geometry and texture placement; physical meter scale is independently verified below. This is not a pixel-parity claim.
- [x] Run actual headless ARGoS rendering: ten nonconstant 960×540 frames per run.
- [x] Verify downward Jolt ray within 0.01 meters of the independently derived floor.
- [ ] Inspect interactive viewer — unavailable: neither DISPLAY nor WAYLAND_DISPLAY is set.
- [x] Verify a supported foot-bot pose in actual Jolt; original generated experiment remains robot-free.
- [x] Copy NOTICE and the full upstream license into the generated bundle.
- [x] Independent whole-branch review and follow-up code review; confirmed findings reproduced and corrected. Code verdict: OK with notes.
- [x] Complete final verification and parent acceptance disposition: headless static target accepted with disclosed approximations and the explicit no-display exception.

## Source-derived geometry and poses

`acceptance-output/source-inspection.json` uses the SDK directly, not importer
coordinate helpers. It finds 5,012 visible ordinary source meshes, 1,540
Preview Surface bindings and zero enabled authored colliders. The importer
also expands PointInstancers and uses visual-geometry collision fallback.

Raw SDK root bounds include unsupported Points/authored extent hints. A
second independent check (`supported-source-bounds.json`) uses supported
mesh points and SDK PointInstancer bounds, yielding ARGoS-meter bounds:

- Minimum: `[-5.737591553, -4.381433105, -0.900196381]`
- Maximum: `[3.372072144, 0.912067947, 3.167500610]`

These agree with both reopened GLBs within **1e-5 meters**. The raw root
maximum X was 3.552853906; it is not used to falsely reject the correctly
filtered supported geometry.

Source camera:
`/root/camera01/GEO/renderCam_hrc/renderCam_buffer/renderCam_srt/renderCam`.
Position `[0.494817421, -0.975042665, 1.094701773]`, look-at
`[1.616075863, 1.792697795, 1.381742281]` (three meters along forward).

Floor:
`/root/alab_set01/lab_structure01_0001/structure_base01_0001/GEO/concrete_M_hrc/lab_M_hrc/lab02_M_geo`.
SDK source face 1105 intersects a downward ray from `[0.5, -1, 1]` at
`[0.5, -1, -0.000773761]`, distance **1.000773761** meters. Actual Jolt
measured **1.000773** meters, error **−4.493e-7 meters**, zero failures.
Evidence: `engine-smoke/argos.log`, `engine-smoke/smoke.json`, and the
calibrated run's corresponding files.

A validation-only configuration starts an existing foot-bot at
`[0.5, -1, 0.02]` with zero wheel commands. After one simulated second,
actual position is `[0.500225, -1.000080, -0.000348]`.
X drift **0.00022465 meters**, floor-height error **0.000426177 meters**;
both below the explicit **0.01-meter** pose tolerance. The flat cylinder's
finite footprint can sit slightly above the sloping triangle under its
center. Evidence: `robot-pose.argos`, `robot-pose.log`, `robot-poses.csv`;
process exit 0. No robot was added to `generated/alab/alab.argos`.

## Conversion and portability

README command with `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
PXR_WORK_THREAD_LIMIT=1`, texture size 1024, no source modifications:

| Measurement | Result |
| --- | --- |
| USD / NumPy / trimesh / Pillow | 26.8 / 2.5.3 / 5.1.1 / 12.3.0 |
| Conversion wall time | **39:27.15** |
| Peak RSS | **26,333,136 KiB** (~25.1 GiB) |
| Visual GLB | **1,224,465,064 bytes**, 2,739 unique meshes |
| Collision GLB | **696,077,044 bytes**, 2,699 unique meshes |
| Placed occurrences / source records | **12,371** |
| Point instances | **6,569** |
| Visual / collision placed triangles | **26,956,901 / 26,956,901** |
| Visual materials / embedded images | **330 / 330** |
| External GLB buffer/image URIs | **0** |

The engine cooks double-sided collision as **53,913,802 triangles**;
this doubling is intentional. Initial smoke collision cook was **45.3157
seconds**. `bundle-inspection.json` independently verifies reopened counts,
finite vertices, embedded assets, UV attributes and bounds. Both GLBs retain
shared geometry, rather than baking every placement into duplicate meshes.

## Actual rendering and calibration

The default exposure run passes ten 960×540 nonconstant frames, minimum
summed per-channel spatial variance **215.9550**, but the interior is dark.
A validation-only copy uses the equivalent of **`--stops 12`**:
aperture 2, shutter 0.04 seconds, ISO 1600. No meshes or source assets were
changed. This calibrated run passes ten frames with minimum spatial
variance **16,594.7460** and was visually inspected via
`engine-smoke-calibrated/frames/frame_0000000009.png`.
Its wall time is **1:15.60**, peak RSS **11,594,584 KiB** (~11.1 GiB).
Run `--stops 12` when regenerating an inspectable ALab bundle. This is
exposure calibration for approximate USD photometry, not reference-render
parity. Renderer: Filament 1.72.1, headless Vulkan, Mesa llvmpipe 25.2.8 /
LLVM 20.1.2. No hardware GPU or display is required for these recorded checks.

## Documented approximation/omission disposition

Preserved `import-red-1.log` through `import-red-6.log` led to generic
regressions, not ALab-specific backend patches. The user explicitly approved
shorter-diagonal approximation of folded nonplanar quads and ambiguous
four-corner subdivision cages, including genuine planar bow-ties. Warnings
remain strict-fatal; ordinary non-subdivision boundary intersections fail.

Diagnostics from the full successful conversion:

| Code | Count / disposition |
| --- | --- |
| subdivision | 12,357: control cages, no subdivision evaluation |
| twisted_quad | 1,196: shorter-diagonal nonplanar triangulation |
| ambiguous_cage_quad | 1,110: explicitly approved cage approximation |
| degenerate | 30 records, 18,158 dropped zero-area/missing-degenerate triangles |
| geometry | 6 unsupported Points prims: bonsai SPRUCE Generation1–4 and Leaves1–2 |
| area_light / shaping | 51 / 51: finite emitters/cones approximated |
| light_linking | 28: unsupported link/filter relationships |
| dome_light | 1: HDR/skydome omission |
| sun_color | 1: renderer sun uses white |
| collision_fallback | 1: no enabled authored colliders; static visible geometry |

No shader/texture-failure diagnostics appeared in this selected target.
All 330 preview images are embedded; production MDL/EXR fidelity is not
claimed. `bundle-inspection.json` preserves the exact six omitted source
paths. The supported full set/rig renders and collides with these disclosed
limits; excluded characters and optional procedural packages were not used.

## Remaining limits

Interactive viewer inspection is explicitly **unvalidated**, not passed.
Transformed/alternate UV normal bases and renderer-level differing inherited
instance appearance have weaker evidence than focused conversion/export
regressions. No path-tracer, subdivision, arbitrary shader, dynamics,
robot-mapping or omitted Points fidelity is claimed.
