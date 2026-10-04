# OpenUSD Environment Importer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Convert local OpenUSD static environments into self-contained ARGoS photorealism/Jolt bundles, with public ALab full-scene validation.

**Architecture:** Compose and sample USD using `pxr`, convert geometry/materials/lights with a small Python package, and export the asset formats ARGoS already consumes. Preserve repeated geometry through scene graph instances; keep source-specific acceptance instructions separate from the general importer. Neither ARGoS nor the Fuel importer needs a runtime or interface change.

**Tech Stack:** Python, OpenUSD (`usd-core`), NumPy, trimesh, Pillow; argparse, unittest, tempfile, ElementTree, JSON, and subprocess from the standard library; existing Filament and Jolt plugins for opt-in integration checks.

**Spec:** `docs/superpowers/specs/2026-10-02-openusd-importer-design.md`

## Global Constraints

- No OpenUSD C++ dependency, new simulator XML element, Filament shader, or new physics entity is required.
- Use the existing Fuel importer as a behavioral reference; do not import it as a library or refactor it.
- Accept local `.usd`, `.usda`, and `.usdc`; load composed payloads and use authored variant selections.
- ARGoS coordinates are Z-up meters; exported glTF is Y-up. Visual props use `orientation="0,0,90"`; collision meshes use `y_up="true"`.
- Names match `[A-Za-z0-9][A-Za-z0-9_-]*`; sanitized empty source stems become `scene`.
- Default options: purpose `render`, collision `auto`, texture size `1024`, light scale `1`, light range `20` meters, exposure `6` stops.
- Include default purpose plus the selected purpose; exclude guides. Authored collider selection is independent of visual visibility/purpose.
- Texture size zero retains resolution. Light scale/range must be positive; all numeric inputs must be finite.
- Auto collision selection is stage-wide inside the selected roots; enabled unusable authored colliders must not trigger silent visual fallback.
- Bake no dynamics, skeleton, subdivision, arbitrary shader, or procedural evaluation. Report the exclusions/approximations described in the spec.
- XML must be escaped. Output ordering and node IDs must be deterministic and collision-free.
- Write to a temporary sibling directory; preserve any existing bundle on conversion/publication failure. Replacement requires `--overwrite`.
- Arena margin is two meters around combined bounds; use `draw_floor="false"` and no synthetic physics floor.
- ALab uses v2.3.0 structure, v2.2.0 Techvar Assets, frame `1004`, static set `/root/alab_set01`, main rig `/root/lightrig01` (verify paths against the downloaded release).
- ALab downloads and engine validation are opt-in, never part of normal Python tests/builds. The Techvar archive is `9,609,335,555` bytes before extraction.
- Keep large upstream/generated assets untracked; preserve source notices and modification descriptions. Public images require the applicable copyright notice.
- Passing offline tests does not complete ALab acceptance. Report blocked checks as unvalidated, not passing.

## Review Focus

1. Non-finite/singular transforms or huge exponent inputs must fail clearly, not emit NaNs or corrupt a bundle (Tasks 1, 2, 4, 6).
2. Material connection cycles and malformed texture graphs must terminate with a source-path diagnostic rather than recursion/hangs (Task 3).
3. Instance-specific material/color bindings must not alias another instance's cached material; duplicate roots must not duplicate placements (Tasks 1, 3, 5).
4. Export, replacement, and permission failures must preserve prior output, including when publication fails after the old directory was moved (Task 6).
5. GLB sampler, UV orientation, normal-map, and emissive-scale details must survive export/reload, not merely appear in Python material objects (Tasks 3, 5, 7).

## File Map and Execution Setup

Create under `src/plugins/simulator/photorealism/environments/`:

- `import_usd_world.py`: dependency-aware argparse entry point and orchestration.
- `requirements-usd.txt`: only `usd-core`, `numpy`, `trimesh`, and `Pillow`.
- `usd_import/__init__.py`: package marker, no dependency-loading side effects.
- `usd_import/scene.py`: options/context, diagnostics, composition, transforms, occurrence traversal and selection.
- `usd_import/geometry.py`: topology, primvars, analytic primitives, material-face partitioning.
- `usd_import/materials.py`: preview bindings/texture graph and PBR conversion.
- `usd_import/lights.py`: supported lights and documented intensity approximation.
- `usd_import/bundle.py`: scene assembly, collision policy, exporter/XML/report, safe publication.
- `usd_import/tests/support.py`: minimal temporary-stage/mesh helpers actually used by the tests.
- `usd_import/tests/test_scene.py`, `test_geometry.py`, `test_materials.py`, `test_lights.py`, `test_bundle.py`, `test_cli.py`, `test_smoke.py`: unittest coverage owned by corresponding tasks.
- `usd_import/tests/smoke_argos.py`: optional engine smoke runner reusing the existing Jolt ray-check loop functions.
- `alab/README.md`, `alab/NOTICE.md`, `alab/acceptance.md`: reproducible sample setup, notices, and honest validation results.

Modify `src/plugins/simulator/photorealism/README.md` (Environments section), `src/plugins/simulator/photorealism/environments/.gitignore`, and `TODO.md`. No mandatory CMake integration or changes to existing C++ tests.

At execution start, read the spec, use the worktree skill for isolation, preserve unrelated files (an untracked `.pi/` existed during planning), and establish:

```sh
E="$PWD/src/plugins/simulator/photorealism/environments"
python3 -m venv /tmp/argos-usd-venv
PY=/tmp/argos-usd-venv/bin/python
export PYTHONPATH="$E${PYTHONPATH:+:$PYTHONPATH}"
```

Install requirements only after execution is approved and Task 1 creates the requirement file. Capture the actual installed versions in test/acceptance results; the spec does not pin Python dependency versions. Do not interpret missing packages as a successful red test: get the environment working, then observe failures against missing/unimplemented product behavior.

Run each task's focused tests first, then all currently implemented tests:

```sh
"$PY" -m unittest discover -s "$E/usd_import/tests" -p 'test_*.py' -v
```

Every task below is a separate red/green deliverable and commit. Tasks 2, 3 and 4 consume Task 1; Tasks 5–8 are sequential. Do not launch parallel work without an authorized execution method.

## Shared Interfaces

Define these concrete records, not interchangeable dictionaries with undocumented fields:

- In `scene.py`, `Options(source: Path, out: Path=Path('.'), name: str|None=None, time: float|None=None, roots: tuple[str,...]=(), purpose: str='render', collision: str='auto', texture_size: int=1024, light_scale: float=1.0, light_range: float=20.0, stops: float=6.0, strict: bool=False, overwrite: bool=False)`.
- `SceneContext(stage: Usd.Stage, options: Options, name: str, time: Usd.TimeCode, stage_to_argos: np.ndarray, report: dict, cache: dict=field(default_factory=dict))`. `stage_to_argos` is a 4×4 column-vector transform from stage coordinates to Z-up meters. Cache holds conversion-only data and is never serialized.
- `Occurrence(prim: Usd.Prim, key: str, transform: np.ndarray)`. Transform maps local authored coordinates directly to Z-up meters; key identifies a world occurrence, including point-instance index and prototype descendant.
- `GeometryPart(key: str, binding_prim: Usd.Prim, mesh: trimesh.Trimesh, uvs: dict[str,np.ndarray])` in `geometry.py`. Mesh coordinates/normals remain local; all per-vertex UV arrays match the expanded mesh vertices. Binding prim is a face subset or its owning mesh.
- `MaterialData(pbr: PBRMaterial, uv_name: str|None, uv_transform: np.ndarray, gltf_overrides: dict)` in `materials.py`. Transform is 3×3 homogeneous UV transform. Overrides have `samplers` mapping glTF texture-slot names (`baseColorTexture`, `metallicRoughnessTexture`, `normalTexture`, `emissiveTexture`) to `wrapS`/`wrapT` settings, and `extensions` mapping extension names to material payloads such as `KHR_materials_emissive_strength`.
- `Lighting(local: list[dict], sun: dict|None, defaulted: bool)` in `lights.py`. Local fields: `source`, `type`, `position`, `direction` (spots), `intensity`, `falloff`, `color`, `inner_angle`/`outer_angle` (spots), `cast_shadows`. Sun fields: `source`, `direction`, `intensity`, `cast_shadows`.
- `Bundle(visual: trimesh.Scene, collision: trimesh.Scene|None, lighting: Lighting, bounds: np.ndarray, report: dict)` in `bundle.py`. Both scenes remain Z-up meters until export; bounds is `(2,3)` lower/upper Z-up coordinates.
- Report fields: `source`, `options`, `versions`, `time`, `metadata`, `bounds_min`, `bounds_max`, `counts`, `collision_policy`, `lights`, `sources`, `diagnostics`. Counts include `visual_triangles`, `collision_triangles`, `geometry_count`, `material_count`, `occurrence_count`, `point_instances`. Diagnostic fields: `code`, `path`, `message`, `details`.

Use standard dataclasses only for these shared records. No factory/registry/backend abstraction. Unsupported features call `warn(ctx, code: str, path: str, message: str, **details) -> None`; strict mode appends the diagnostic then raises `ValueError`. Fatal data/composition errors raise `ValueError` with source context regardless of strict mode.

---

### Task 1: Composed stage selection, occurrence traversal, and diagnostics

**Files:** Create `requirements-usd.txt`, `usd_import/__init__.py`, `usd_import/scene.py`, `usd_import/tests/support.py`, `usd_import/tests/test_scene.py` under the environment directory.

**Interfaces:** Produces the scene records above, `read_stage(options: Options) -> SceneContext`, `iter_occurrences(ctx: SceneContext) -> Iterator[Occurrence]`, `is_visual(occ: Occurrence, ctx: SceneContext) -> bool`, `is_collider(occ: Occurrence, ctx: SceneContext) -> bool`, and `warn(...)`. `iter_occurrences` yields typed renderable/light prims, including unsupported renderable types so downstream converters can diagnose them; it skips inactive/class prims, not invisible authored colliders.

- [ ] **Step 1: Create the four-dependency requirement file and install it into the isolated environment.** Run `"$PY" -m pip install -r "$E/requirements-usd.txt"`; verify `"$PY" -c 'from pxr import Usd; import numpy, trimesh, PIL; print(Usd.GetVersion())'` exits zero. These are the only Python product dependencies.
- [ ] **Step 2: Write failing `SceneTests` and the temporary fixture helpers.** `make_stage(test: unittest.TestCase, *, axis='Z', meters_per_unit=1.0, start: float|None=None) -> Usd.Stage` registers cleanup on the test. `context(stage: Usd.Stage, **options) -> SceneContext` saves the stage then calls `read_stage(Options(source=Path(stage.GetRootLayer().realPath), **options))`. `author_mesh(stage: Usd.Stage, path: str, points: list, faces: list[list[int]]) -> UsdGeom.Mesh` defines topology with subdivisionScheme `none`.

```python
def test_y_up_centimeters(self):
    stage = make_stage(self, axis='Y', meters_per_unit=0.01)
    UsdGeom.Cube.Define(stage, '/World/Box')
    ctx = context(stage)
    np.testing.assert_allclose(
        ctx.stage_to_argos @ [0, 100, 0, 1], [0, 0, 1, 1], atol=1e-12)
    self.assertEqual(len(list(iter_occurrences(ctx))), 1)
```

Also assert: metadata defaults are Y-up/0.01; a reset child under a translated parent retains only its own translation; authored start time `1004` and explicit time `1005` select different point samples; duplicate/ancestor-overlapping roots give one occurrence; nonexistent/relative/property roots fail. Native instances give distinct occurrence keys. Point instances honor prototype transforms and masks, do not emit prototype geometry separately, and invalid prototype indices fail. Enabled hidden proxy colliders are retained; CollisionAPI on an ancestor is not inferred on children. Missing referenced/payload layers fail with the missing path. Add Review Focus 1 finite-option/matrix tests and Review Focus 3 root deduplication tests.

- [ ] **Step 3: Run `"$PY" -m unittest discover -s "$E/usd_import/tests" -p test_scene.py -v`.** Expect red on missing scene functions/behavior, not missing third-party dependencies.
- [ ] **Step 4: Implement `scene.py` and the exact records/signatures.** Open with payloads loaded; detect composition errors with SDK diagnostics/composition-error APIs. Pick explicit time, else authored start time, else default. Use USD xform caches and instance-proxy traversal; transpose Gf matrices once into NumPy column-vector convention. Use SDK point-instancer transforms/masks, include each prototype descendant transform once, exclude prototype targets from normal traversal, and preserve instancer visibility plus appropriate prototype composition. Sort by occurrence key. Visibility/purpose only affect `is_visual`; `is_collider` uses applied CollisionAPI and composed enabled value. Validate names/numeric settings, roots, and finite/invertible transforms.
- [ ] **Step 5: Run the focused command and all current tests.** Expect zero failures; inspect recorded metadata and diagnostic paths.
- [ ] **Step 6: Commit the Task 1 files:** `git commit -m 'feat: read composed USD snapshots and instances'` after staging only those paths.

### Task 2: Topology, primvars, analytic shapes, and face-material partitioning

**Files:** Create `usd_import/geometry.py` and `usd_import/tests/test_geometry.py`.

**Interfaces:** Consumes scene context/occurrences/diagnostics. Produces `GeometryPart` and `convert_geometry(occ: Occurrence, ctx: SceneContext) -> list[GeometryPart]`. Parts preserve source material subsets and local attributes but contain no bound textures yet. Unsupported visible geometry returns no parts with a diagnostic; an enabled authored collider with unsupported geometry is fatal.

- [ ] **Step 1: Write failing `GeometryTests`.** The primary test proves a concave polygon is not incorrectly fan-triangulated:

```python
def test_concave_face_area(self):
    stage = make_stage(self)
    author_mesh(stage, '/Mesh',
                [[0,0,0], [2,0,0], [2,2,0], [1,1,0], [0,2,0]],
                [[0,1,2,3,4]])
    ctx = context(stage)
    parts = convert_geometry(next(iter_occurrences(ctx)), ctx)
    self.assertEqual(sum(len(p.mesh.faces) for p in parts), 3)
    self.assertAlmostEqual(sum(p.mesh.area for p in parts), 3.0)
```

Add exact checks for skipped hole faces, reversed left-handed winding, face-varying UV seams with indices, uniform/vertex/indexed normals, inherited UV/display-color primvars, disjoint face material subsets with remaining faces, and invalid overlapping subset/index/attribute cardinality diagnostics. Check cube size 2 gives bounds `[-1,1]` on each axis, and sphere/cylinder/cone/capsule honor their authored dimensions/axis. Zero-area faces produce a diagnostic; nonempty surviving geometry remains valid. Invalid topology or singular/non-finite transform fails with its prim path. Subdivision cages and unsupported curves/skin/procedural types produce diagnostics; strict mode fails.

- [ ] **Step 2: Run `"$PY" -m unittest discover -s "$E/usd_import/tests" -p test_geometry.py -v`.** Expect failures against missing conversion behavior.
- [ ] **Step 3: Implement `convert_geometry` in `geometry.py`.** Use trimesh creation APIs for primitives. For general faces, project onto the dominant Newell-normal plane and ear-clip preserving face-corner indices; fail on self-intersection/untriangulable input rather than emitting wrong triangles. User-approved execution amendment: try alternate projections for nonplanar faces; folded nonplanar quads with no usable orthogonal projection use their shorter 3D diagonal with a diagnostic, after rejecting actual 3D boundary-edge intersections. Planar self-intersections remain fatal for non-subdivision meshes. Second user-approved amendment: ambiguous four-corner subdivision cages may use a shorter-diagonal approximation with a strict-fatal `ambiguous_cage_quad` diagnostic, including genuinely planar bow-ties. Mark the O(n²) per-face ceiling with a `ponytail:` comment. Use USD primvar flattening/interpolation and split corner vertices before building unprocessed trimesh meshes. Preserve valid normals or generate them; copy face material subsets into separate parts without duplicate triangles. Retain both original `st` and alternate named UV primvars for later material selection.
- [ ] **Step 4: Run focused and all current tests.** Expect all green and matching area/bounds/cardinality assertions.
- [ ] **Step 5: Commit:** `git commit -m 'feat: convert USD topology and geometric attributes'` with only Task 2 files staged.

### Task 3: Preview materials, texture graph, and GLB-visible material metadata

**Files:** Create `usd_import/materials.py` and `usd_import/tests/test_materials.py`.

**Interfaces:** Consumes scene diagnostics and geometry-part binding prims. Produces `MaterialData` and `convert_material(binding_prim: Usd.Prim, ctx: SceneContext) -> MaterialData`. `attach_material(part: GeometryPart, data: MaterialData, ctx: SceneContext) -> trimesh.Trimesh` returns a mesh with selected/transformed UVs and PBR data; stores the export overrides in mesh metadata under `usd_gltf_overrides`. It must not mutate another instance's cached mesh.

- [ ] **Step 1: Write failing `MaterialTests`.** Author a PreviewSurface with `diffuseColor=(0.2,0.4,0.6)`, `metallic=0.25`, `roughness=0.75`, `opacity=0.5`; assert converted factors match and alpha mode is BLEND. Set opacityThreshold `0.3`; assert MASK and alpha cutoff `0.3`. Author conflicting universal/preview bindings and assert the preview material wins. Bind a subset differently and assert its material does not leak onto the remaining faces.

Define test-only `preview_fixture(test: unittest.TestCase, **inputs) -> tuple[SceneContext, Usd.Prim]` to create a cube bound to a UsdPreviewSurface and return its context/binding prim. Pin constant conversion with:

```python
def test_preview_constants(self):
    ctx, prim = preview_fixture(self, diffuseColor=(0.2,0.4,0.6),
                                metallic=0.25, roughness=0.75, opacity=0.5)
    data = convert_material(prim, ctx)
    self.assertAlmostEqual(data.pbr.metallicFactor, 0.25)
    self.assertAlmostEqual(data.pbr.roughnessFactor, 0.75)
    self.assertEqual(data.pbr.alphaMode, 'BLEND')
```

Create a 2×2 diagnostic texture with different corner colors, author it in a referenced layer directory, and author a `perfuv` reader plus Transform2d. Assert the resolved image is correct, `perfuv` is chosen over `st`, transformed UVs match the authored transform, and shared image decoding does not duplicate work. Test roughness R=64 and metallic B=192 inputs pack glTF G/B to approximately those byte values; normal maps with USD signed scale/bias re-encode correctly to glTF tangent-space RGB. Test raw/sRGB conversion, opacity channel, repeat/clamp/mirror wrap, asymmetric resizing (2048×1024 -> 1024×512), texture-size zero, and single-tile versus multi-tile UDIM. Strong emission must record an emissive-strength override rather than silently clamp away brightness. Add Review Focus 2 cycles and unknown shader nodes; Review Focus 3 instance-specific binding/color differences; Review Focus 5 sampler/normal/emissive data needed by the exporter. Missing texture warns and falls back; strict mode raises.

- [ ] **Step 2: Run `"$PY" -m unittest discover -s "$E/usd_import/tests" -p test_materials.py -v`.** Expect red on missing functions or factor/texture expectations.
- [ ] **Step 3: Implement the exact interfaces in `materials.py`.** Prefer preview bindings, then universal; resolve shader output in the corresponding context with universal fallback. Read constants or the explicitly supported UV graph; track visited connections to bound traversal. Resolve asset paths via composed USD asset values/authoring layer, use Pillow for decoding and bake channel scale/bias/color-space/compatible texture operations into images. Keep unsupported multiple-UV/texture-transform combinations explicit. Use texture/material caches scoped to the context and keyed by resolved content/settings, not merely occurrence/prototype identity. Build normal/metallic-roughness/alpha/emissive textures; put sampler and emissive-strength extension information in `gltf_overrides`. Fall back to expanded display color/opacity then neutral material, with diagnostics for incompatible bound shaders.
- [ ] **Step 4: Run focused and all current tests.** Require exact factors, material separation, expected texture pixels/UVs, and terminating graph-cycle diagnostics.
- [ ] **Step 5: Commit:** `git commit -m 'feat: translate USD preview materials and textures'` with Task 3 files staged.

### Task 4: Supported lights and inspectable lighting defaults

**Files:** Create `usd_import/lights.py` and `usd_import/tests/test_lights.py`.

**Interfaces:** Consumes context/occurrences; produces `Lighting` and `convert_lights(ctx: SceneContext, occurrences: list[Occurrence]) -> Lighting`.

- [ ] **Step 1: Write failing `LightTests`.** A sphere light with intensity `10`, exposure `2`, and light_scale `3` must produce `120` lumens; light_range `20` gives falloff `20`. Translate/rotate lights under a Y-up centimeter stage and assert their position/direction agree with the Task 1 frame. A shaped sphere cone angle `30` yields outer angle `30`; choose inner angle equal to outer angle when softness is zero, and record nonzero softness/focus as an approximation. Rectangle/disk/cylinder become local approximations. Choose the strongest distant light and use source-key sorting to break ties. Dome/link/filter/normalization/emitter-size approximations are recorded and strict-fatal. Color temperature `6500` must be near white; lower values must be warmer. Zero-intensity or invisible lights do not suppress no-usable-light defaults. NaN, negative intensity, zero direction, and overflowing exposure fail clearly (Review Focus 1).
```python
def test_intensity_exposure_and_scale(self):
    stage = make_stage(self)
    lamp = UsdLux.SphereLight.Define(stage, '/Lamp')
    lamp.CreateIntensityAttr(10)
    lamp.CreateExposureAttr(2)
    ctx = context(stage, light_scale=3)
    lighting = convert_lights(ctx, list(iter_occurrences(ctx)))
    self.assertEqual(lighting.local[0]['intensity'], 120)
    self.assertEqual(lighting.local[0]['falloff'], 20)
```

- [ ] **Step 2: Run `"$PY" -m unittest discover -s "$E/usd_import/tests" -p test_lights.py -v`.** Expect missing light-conversion/default behavior to fail.
- [ ] **Step 3: Implement `lights.py`.** Use the composed sphere/rect/disk/cylinder/distant schemas and ShapingAPI; local forward is USD -Z transformed/normalized in ARGoS coordinates. Use `intensity * 2**exposure * light_scale` in lumens or lux, with finite validation. Apply a bounded standard blackbody color approximation and record it rather than add a color-science dependency. Default scene illumination when no usable lights exist is sun direction `(0.6,0.2,-1)`, intensity `100000`, shadows true, and sky color `(0.53,0.71,0.92)` (matching plugin usage); mark it as generated, not authored. Clamp spot angles only to the existing medium's supported range and diagnose any change. Include authored normalization/size/softness/focus in approximation details.
- [ ] **Step 4: Run focused and all current tests.** Expect green with numeric intensity and frame assertions.
- [ ] **Step 5: Commit:** `git commit -m 'feat: map USD lights into photorealism settings'` with Task 4 files staged.

### Task 5: Shared visual/collision scenes and validated GLB export

**Files:** Create `usd_import/bundle.py`, `usd_import/tests/test_bundle.py`.

**Interfaces:** Consumes Tasks 1–4. Produces `Bundle`, `build_bundle(ctx: SceneContext) -> Bundle`, and `export_glb(scene: trimesh.Scene, path: Path) -> None`. Export writes a Y-up GLB from a copied Z-up scene, including Task 3 overrides, without changing the original scene or relying on external texture files.

- [ ] **Step 1: Write failing `BundleTests`.** Author a visible box and a hidden collider below Z=0; assert auto uses the collider, visual uses the visible box, none gives `collision is None`, authored-with-no-colliders raises, and unsupported enabled colliders do not cause visual fallback. Default auto-with-no-colliders reports `collision_policy='visual-fallback'`. Geometry in overlapping roots counts once. Two translated instances reuse one geometry while preserving two node placements; different resolved materials/UVs do not reuse an incompatible mesh.

```python
def test_exported_scene_maps_back_to_argos(self):
    stage = make_stage(self, axis='Y', meters_per_unit=0.01)
    cube = UsdGeom.Cube.Define(stage, '/Box')
    cube.AddTranslateOp().Set((100, 0, 0))
    bundle = build_bundle(context(stage))
    original = bundle.visual.bounds.copy()
    path = Path(stage.GetRootLayer().realPath).parent / 'visual.glb'
    export_glb(bundle.visual, path)
    loaded = trimesh.load(path, force='scene')
    loaded.apply_transform(trimesh.transformations.rotation_matrix(
        np.pi / 2, [1, 0, 0]))
    np.testing.assert_allclose(loaded.bounds, original, atol=1e-6)
    np.testing.assert_allclose(bundle.visual.bounds, original)
```

Reload both exported scenes and compare node transforms/triangle counts/materials. Inspect the actual GLB JSON and embedded image bytes for UV orientation, named alternate UV selection, alpha mode/cutoff, sampler wraps, normal texture and emissive-strength extension (Review Focus 5). Verify inverse-transpose normals under nonuniform scales, consistently oriented mirrored instances, and collision GLB has no images/textures. Simulate format-size overflow with mocked exporter buffers; require a descriptive failure before publication, not a real multi-GB unit fixture.

- [ ] **Step 2: Run `"$PY" -m unittest discover -s "$E/usd_import/tests" -p test_bundle.py -v`.** Expect missing scene assembly/export or round-trip assertions to fail.
- [ ] **Step 3: Implement `build_bundle` and `export_glb` in `bundle.py`.** Traverse once; retain enabled collider count even for unsupported types to enforce auto policy. Convert selected geometry, attach materials only for visual parts, and convert lights. Insert cached geometry with per-occurrence transforms; key reuse by compatible geometry/attributes/material/settings. For reflection handling, bake a fixed reflection `F=diag(-1,1,1,1)` into a cached reflected mesh and replace negative-determinant node transform `T` with `T @ F`; reverse winding and transform normals for the baked reflection. This leaves node determinants positive while retaining reuse. Keep positive affine transforms on nodes for Filament/Jolt. Compute combined Z-up bounds/counts/source records before export. Rotate the exported scene globally -90 degrees around X. Use trimesh's glTF exporter postprocessing hooks for missing sampler/material extensions and reject size-limit violations. Do not implement a second complete glTF exporter.
- [ ] **Step 4: Run focused and all current tests.** Require GLB reload/material/normal/placement checks to pass, not merely file existence.
- [ ] **Step 5: Commit:** `git commit -m 'feat: assemble USD scenery and static collision bundles'` with Task 5 files staged.

### Task 6: CLI, portable XML/report, and failure-safe publication

**Files:** Modify `usd_import/bundle.py`, `usd_import/tests/test_bundle.py`; create `import_usd_world.py`, `usd_import/tests/test_cli.py`; modify the plugin README and environments `.gitignore`.

**Interfaces:** Consumes `read_stage`, `build_bundle`, `export_glb`. Produces `write_bundle(bundle: Bundle, ctx: SceneContext) -> Path` and entry-point `main(argv: list[str]|None=None) -> int`. Success is exit 0; argument errors exit 2; conversion/publication errors exit 1 with concise stderr. `--help` must work without `pxr` installed.

- [ ] **Step 1: Write failing CLI/publication tests.** A subprocess run against a fixture produces the visual GLB, collision GLB, lights XML, `.argos`, and JSON; reload all referenced files from another working directory. For none collision, no mesh/file appears. Check medium ID `pr`, backend Vulkan, floor drawing false, prop orientation `0,0,90`, mesh y_up/double_sided true, Jolt gravity `9.81`, and no floor plugin. Arena bounds equal combined bounds ±2 meters; clamp any resulting dimension to at least `0.001` meters. Exposure six stops gives f/2, 1/100 s, ISO100; capture viewport/debug camera settings separately from authored USD cameras.

```python
def test_generated_configuration_uses_existing_plugins(self):
    stage = make_stage(self)
    UsdGeom.Cube.Define(stage, '/Box')
    stage.GetRootLayer().Save()
    source = Path(stage.GetRootLayer().realPath)
    out = source.parent / 'bundles'
    self.assertEqual(main([str(source), '--out', str(out), '--name', 'scene']), 0)
    tree = ET.parse(out / 'scene' / 'scene.argos')
    self.assertEqual(tree.find('./media/photorealism').get('draw_floor'), 'false')
    self.assertEqual(tree.find('./arena/mesh').get('y_up'), 'true')
    self.assertIsNone(tree.find('./physics_engines/jolt/floor'))
```

Assert hostile source names/labels are escaped in XML and rejected as explicit path stems. Reject `../outside`, empty names, negative texture size, zero/NaN/Inf light settings, invalid roots and USDZ. Output options/counts/diagnostics serialize to JSON without NaN. Simulate missing `pxr`: help exits 0, import fails with install guidance, not a traceback. Repeat successful conversion and compare structural output for deterministic IDs/order.

For Review Focus 4, prepopulate an existing bundle with a sentinel, then inject errors at export, old-directory rename, and new-directory rename. Require the old bundle/content restored, no partial replacement, and no silently swallowed publication error. Separately inject a backup-cleanup failure after successful publication: require the new bundle usable, old backup retained, and a clear cleanup warning. Existing bundle without overwrite fails before expensive conversion; destination symlinks must not cause deletion of unrelated directories. Strict warnings leave no bundle.

- [ ] **Step 2: Run `"$PY" -m unittest discover -s "$E/usd_import/tests" -p test_cli.py -v` and repeat with `-p test_bundle.py`.** Expect red against missing CLI/writer/publication behavior.
- [ ] **Step 3: Implement `write_bundle` in `bundle.py`.** Use ElementTree and JSON; keep all asset references bundle-relative, never insert USD labels into XML comments. Generate a no-robot standalone experiment, viewer/debug camera, local lights XML and complete report. Normal generated experiment length is `0` (interactive); smoke tests override it to a finite duration. Set camera fov `70`, debug resolution `960,540`, viewer `1280,720`, debug period `10`, near `0.1`, far `max(300, 2*scene_diagonal)` and bounds-based pose. When converted local lights exist but no translated distant light does, explicitly write sun intensity zero and black sky so plugin defaults do not add an unrequested sun; use the Task 4 sun/sky only when `lighting.defaulted` is true. Use the Fuel exposure formula, without importing its module. Export/validate in a sibling temporary directory; for overwrite rename old output to a backup, rename new output into place, and roll back on failure. Never delete an unrelated directory or follow a destination symlink during replacement. Backup-cleanup errors must preserve usable output and report the retained backup.
- [ ] **Step 4: Implement `main` and usage documentation.** Parse with stdlib before importing product dependencies, instantiate the exact Options fields, reject invalid output destinations early, then call reader/assembler/writer. Report diagnostics and final path concisely. Document the isolated install, basic command, collision choices, purpose/time/root selection, light calibration, strict mode and supported limits. Ignore Python caches and `alab/{downloads,source,generated,acceptance-output}/`; retain source/notice/documentation files. Do not ignore unrelated existing files.
- [ ] **Step 5: Run focused and all current tests, `"$PY" "$E/import_usd_world.py" --help`, and `git diff --check`.** Expect zero failures, useful help and clean diff. Run a standalone synthetic import outside the test process and reopen its assets.
- [ ] **Step 6: Commit:** `git commit -m 'feat: expose portable USD environment import command'` after staging only CLI/writer/tests/docs/ignore changes.

### Task 7: Optional real ARGoS rendering and Jolt ray smoke check

**Files:** Create `usd_import/tests/smoke_argos.py`, `usd_import/tests/test_smoke.py`.

**Interfaces:** Consumes a completed bundle and existing engine/test binaries. Produces `run_smoke(config: Path, argos: Path, mesh_library: Path, ray_origin: tuple[float,float,float], expected_distance: float, camera_position: tuple[float,float,float], camera_look_at: tuple[float,float,float], output: Path) -> dict`. Its CLI accepts matching `--config`, `--argos`, `--mesh-library`, `--ray-origin`, `--expected-distance`, `--camera-position`, `--camera-look-at`, `--output`. Ray direction is fixed downward `(0,0,-1)` for the verified floor check. Failed external execution/frame validation raises `RuntimeError` with log context.

- [ ] **Step 1: Write failing smoke utility tests.** Mock subprocess only for unit checks: assert prepared config uses absolute resolved bundle asset paths, finite experiment length `1`, 10 ticks/second, debug period `1`, specified camera pose and a mesh ray expectation with tolerance `0.01` meters. Assert invocation uses `argos -z -c`, inherits ARGOS_PLUGIN_PATH/Vulkan environment, removes visualization only in the test copy, and propagates nonzero exit/no frames/all-black or constant frames as failures. The original config/assets must remain unchanged.
Define test-only `make_smoke_config(test: unittest.TestCase) -> Path` by importing a tiny floor/wall fixture through Task 6. Pin subprocess failure handling without requiring installed ARGoS:

```python
def test_failed_argos_is_not_a_pass(self):
    config = make_smoke_config(self)
    argos, library = config.parent / 'argos', config.parent / 'mesh-loops.so'
    argos.write_text('stub')
    argos.chmod(0o755)
    library.write_text('stub')
    with mock.patch('smoke_argos.subprocess.run', side_effect=
                    subprocess.CalledProcessError(1, ['argos'])) as execute:
        with self.assertRaises(RuntimeError):
            run_smoke(config, argos, library,
                      (0,0,1), 1.0, (1,1,1), (0,0,0),
                      config.parent / 'smoke-output')
        execute.assert_called_once()
```

- [ ] **Step 2: Run `"$PY" -m unittest discover -s "$E/usd_import/tests" -p test_smoke.py -v`.** Expect missing runner/prepared-config assertions to fail.
- [ ] **Step 3: Implement the runner.** Reuse `src/testing/jolt/mesh/loop_functions.cpp` via built `libmesh_jolt_loop_functions` and label `mesh_loop_functions`; add one `<ray>` with source expectation and a finite ray length exceeding expected distance. This is already a Jolt-backed embodied-entity ray check. Do not create a new C++ plugin. Store logs/prepared XML/frames in the output directory and check PNG dimensions, nonzero pixels and nonzero RGB variance using Pillow. Return config/ray/frame/log metadata in a JSON-serializable result.
- [ ] **Step 4: Run focused and all current offline tests.** Expect no engine dependencies or downloads in these unittest runs.
- [ ] **Step 5: Run the opt-in runner against a synthetic exported floor/wall bundle when plugins are available.** Locate/provide an ARGoS build and its `libmesh_jolt_loop_functions`, exporting that build's ARGOS_PLUGIN_PATH. No argos3 executable was on PATH at planning time. If building is needed, use the existing simulator/testing configuration with `ARGOS_BUILD_JOLT=ON` and a valid FILAMENT_DIR; do not invent a testing build option. Expected results: ARGoS exits zero, ray check reports zero failures, and nonconstant PNG frames exist. Missing plugin/driver/build is an explicit blocked check, not a skipped pass.
- [ ] **Step 6: Commit:** `git commit -m 'test: add optional USD bundle render and collision smoke check'` with runner/tests only.

### Task 8: Downloaded ALab target, reproducible guide, and final acceptance

**Files:** Create `alab/README.md`, `alab/NOTICE.md`, `alab/acceptance.md`; modify plugin README and TODO.md. Do not commit upstream archives/source, generated binary assets, or frames.

**Interfaces:** Consumes Tasks 1–7. Uses the source/package/version URLs in the approved spec, the importer CLI and smoke runner without adding an ALab-specific importer/backend.

- [ ] **Step 1: Write the ALab acceptance checklist in `alab/acceptance.md` with every item initially unchecked.** Require versioned archive hashes, package layout and root-path validation, composed preview materials, both GLBs reopened, exclusion of objects outside the selected roots, interior texture/scale inspection, headless frames, downward Jolt floor ray, viewer inspection, and a verified robot starting pose. A file's presence alone cannot satisfy a check. Add the licensed-source/modified-output notice requirements.
- [ ] **Step 2: Establish opt-in and inspect/download the two archives outside tracked output.** Do not download during normal tests; ask for download permission if execution approval did not explicitly include this 9.6 GB target run. Check available disk/RAM/SDK/driver first. Use resumable `curl -L --fail -C - -o ...` or `wget -c` with the exact versioned URLs from the spec, calculate SHA-256, and inspect archive listings before extraction. Preserve every upstream notice. Extract only verified relative paths into `alab/source/`; merge the Techvar fragment directory into the release's ALab directory according to its inspected layout. Do not assume the structure-only package contains geometry.
- [ ] **Step 3: Run the full-scene red check before fixing source-specific incompatibilities.** Confirm frame 1004, selected set/rig prims and resolved dependencies via `pxr`, then run:

```sh
"$PY" "$E/import_usd_world.py" "$ALAB_SOURCE/entry.usda" \
  --name alab --out "$E/alab/generated" --time 1004 \
  --root /root/alab_set01 --root /root/lightrig01 \
  --purpose render --collision auto --texture-size 1024
```

`ALAB_SOURCE` must point to the inspected merged directory containing entry.usda. Initially expect this run to uncover incompatibilities or the absence of verified camera/floor checks; record actual failures. Do not use strict mode: documented area/subdivision/lighting approximations are expected. If the release's actual paths differ, record/validate them and use the corrected sample command; do not silently broaden selection to animated characters.

- [ ] **Step 4: Resolve reproducible importer defects revealed by ALab with small offline red regressions first.** Fix the owning module once, then rerun its tests and all offline tests. Missing sample packages, authored variant prerequisites or incompatible production shader bindings belong in setup instructions, not hard-coded importer paths or global special cases. If a material/geometry requirement would expand the approved scope, stop and request a spec revision instead of silently falling back and calling it complete.
- [ ] **Step 5: Re-run and inspect the actual imported assets.** Reopen GLBs; compare Z-up combined bounds with USD-computed bounds for selected roots; verify positive triangle/material counts, embedded JPEG textures, material purpose/UV selection, and source records restricted to the chosen roots. Inspect diagnostic list for unexpected omissions. Record CLI, versions, source hashes, runtime, peak memory if available, scene counts and file sizes.
- [ ] **Step 6: Choose and verify an interior camera and floor ray.** Inspect selected geometry, identify a clear interior floor and world coordinates, run Task 7 with its expected source-derived floor distance, then inspect the actual PNG and interactive viewer. Tune --light-scale/--stops using the exposed knobs; record values and approximations. Require Jolt's downward ray expectation within `0.01` meters. Record a supported clear-space robot starting pose, not simply the center of bounds. If no interactive display is available, explicitly leave that item unvalidated.
- [ ] **Step 7: Write the sample README/NOTICE and truthful acceptance result.** Include verified download/merge/import commands, proxy-purpose alternative, license terms/notices and modification description, interior camera configuration and finite headless command, sensor medium `pr`, starting pose, documented omissions, and measured results. Explain that the generated experiment has no robots by default. Do not label the sample working until the full target checks are satisfied. Keep failed/blocked statuses visible if resources prevent them.
- [ ] **Step 8: Run final verification and review the whole branch.** Run all offline tests, standalone CLI example/help, optional engine smoke, ALab acceptance, and `git diff --check`. Review every spec requirement against the task checkboxes and actual evidence. Independent whole-branch review uses the execution method authorized by the user; address feedback with verified regression checks. No reviewer is dispatched before that authorization.
- [ ] **Step 9: Commit the ALab guide/notices/results and task status:** `git commit -m 'docs: add ALab USD environment target and acceptance results'`. If acceptance is blocked, use a commit message describing setup/unvalidated status rather than successful validation.

## Plan Self-Review and Handoff

- Coverage: Tasks 1–2 cover snapshot/composition/instances/topology; Task 3 shading; Task 4 lighting; Tasks 5–6 collision/output/errors; Task 7 engine checks; Task 8 public target/license/resources and completion evidence.
- Signature consistency: all cross-task records/functions are defined above; test helpers and the smoke result are defined in their owning tasks.
- Review Focus: each listed failure mode has an owning red test, including export-visible material settings and replacement rollback.
- Proportion: code bodies are left to implementation; tests pin values and outputs rather than transcribing the importer.
- Execution recommendation: **Native**. The eight tasks share the same scene/material/report contracts, so keeping one implementation context avoids repeated handoff cost; one independent whole-branch review can examine their interactions at the end. Subagent-driven implementation remains available if the user prefers per-task independent reviews.

No product implementation, dependency installation, or ALab download is authorized by merely writing this plan. Obtain the user's plan review and execution-method choice before proceeding.
