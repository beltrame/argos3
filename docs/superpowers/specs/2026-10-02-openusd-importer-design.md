# OpenUSD static environment importer

## Intent and approved scope

Provide an offline importer that lets ARGoS robots inhabit a standard OpenUSD
environment, using the existing photorealism plugin for images and Jolt for
static collisions. The user selected static environments rather than visual-only
scenery or a full USD dynamics/robot importer, approved Python conversion to the
existing asset formats, and selected the public ALab scene as the full-scene
acceptance target.

Success means an import produces a self-contained environment directory that
ARGoS can load without OpenUSD installed. The supplied sample instructions must
lead from a public ALab download to an interactive and headless ARGoS experiment.
Small offline fixtures provide automated regression coverage; they do not replace
the full-scene acceptance run.

## Architecture and integration

Use the OpenUSD Python API (`pxr`, supplied by `usd-core`) to compose the stage.
Use NumPy, trimesh, and Pillow to prepare geometry, PBR materials, and binary glTF.
Dependencies live in a separate Python environment. No OpenUSD C++ dependency,
new simulator XML element, Filament shader, or new physics entity is required.

The public entry point is
`src/plugins/simulator/photorealism/environments/import_usd_world.py`. Keep the
conversion code in a sibling `usd_import` package with these boundaries:

- **Scene reader:** opens the composed stage, selects the snapshot and roots,
  traverses instances, and supplies transforms, visibility, purpose, and source
  identity.
- **Geometry converter:** validates and triangulates geometry, expands indexed
  attributes, converts frames, and separates visual and collision membership.
- **Material converter:** resolves preview bindings and the supported texture
  graph into glTF PBR data, caching textures and materials.
- **Light converter:** turns supported USD lights into the existing sun and
  point/spot configuration with explicit approximation diagnostics.
- **Bundle writer:** exports GLBs, XML, experiment configuration, and the report.

Use the existing Fuel importer as a behavioral reference. Do not import it as a
library, which would introduce unrelated COLLADA dependencies, and do not refactor
it as part of this work.

Existing integration points:

- `<photorealism><scenery><prop>` loads the visual GLB.
- `<arena><mesh>` loads the collision GLB through Jolt.
- `<photorealism><lights>`, `<sun>`, and `<exposure>` configure illumination.
- The photorealism medium's existing scenery identity/depth handling is retained;
  USD semantic labels are not imported as per-object segmentation annotations.

## Command-line contract

Basic usage:

```sh
python import_usd_world.py /path/to/scene.usda --out environments --name scene
argos3 -c environments/scene/scene.argos
argos3 -z -c environments/scene/scene.argos
```

Options:

- `--out`: output parent directory, default current directory.
- `--name`: directory/file stem matching `[A-Za-z0-9][A-Za-z0-9_-]*`; default
  sanitized source stem, using `scene` if sanitization yields no name. Reject an
  explicit empty name, path separators, and traversal components.
- `--time`: numeric USD time code; absent means the authored stage start time
  when animated, otherwise USD default time. The chosen value is recorded.
- `--root`: repeatable absolute prim path. Absent means the entire stage. Roots
  include their descendants; overlapping roots do not duplicate output.
- `--purpose {render,proxy}`: include default purpose plus the selected purpose;
  default render. Exclude guide geometry. Collision selection is independent of
  visual visibility and purpose.
- `--collision {auto,authored,visual,none}`: default auto, defined below.
- `--texture-size`: maximum dimension, default 1024; zero retains resolution.
- `--light-scale`: positive multiplier on converted light intensities, default 1.
- `--light-range`: positive cutoff radius in meters for local lights, default 20.
- `--stops`: exposure stops opened from sunny 16, default 6, applied to generated
  experiment settings; use the existing Fuel importer's exposure convention.
- `--strict`: fail on unsupported features, approximations, or missing assets
  instead of producing a partial conversion.
- `--overwrite`: explicitly permit replacing an existing generated bundle.

Accept local `.usd`, `.usda`, and `.usdc` stages and locally resolvable layers,
references, payloads, and texture files. Load payloads. Use authored variant
selections; no new variant editing interface is required. USDZ and remote asset
resolver plugins are outside this version.

## Scene and geometry fidelity

Honor composed transforms, reset-xform stacks, inherited visibility and purpose,
stage `metersPerUnit`, and Y/Z up axes. Use USD metadata defaults when absent and
record the effective values. ARGoS coordinates are Z-up meters; exported glTF is
Y-up. The visual prop uses `orientation="0,0,90"`; the collision mesh uses its
existing `y_up="true"` behavior. Geometry, cameras used for viewing, and light
positions/directions must share the same conversion.

Support `UsdGeomMesh` and cube, sphere, cylinder, cone, and capsule primitives.
Triangulate polygon faces, including concave faces; respect hole indices and
left-handed orientation. Reject invalid indices and malformed topology with a
source-path diagnostic. Preserve authored normals, including indexed/interpolated
normals, when valid; otherwise generate normals. Nonuniform scales use the inverse
transpose for normals. Reflections must preserve consistent winding. Drop
zero-area triangles with a counted diagnostic.

Support constant, uniform, vertex/varying, and face-varying UV and display-color
attributes, including indexed primvars. Split vertices as necessary to preserve
face-corner attributes and material boundaries. Handle face material subsets
without duplicating covered faces.

Support native USD instances and static `UsdGeomPointInstancer` placements at the
selected snapshot, including instance masks and prototype transforms. Do not
also export instance prototypes as world objects. Retain shared glTF geometry
where possible rather than duplicating the entire asset for each placement.

Subdivision surfaces are exported as triangulated control cages with a diagnostic;
subdivision evaluation, displacement, curves, volumes, and skinning are excluded.
A sampled mesh point cache is valid static input. Do not claim to evaluate a
skeleton or procedural geometry. Hidden or disabled visual objects are excluded.
An empty visual result is fatal.

## Materials and textures

Resolve composed material bindings with the `preview` material purpose first,
falling back to the universal binding. Use bound `UsdPreviewSurface` shaders.
Convert diffuse/base color, opacity and opacity threshold, metallic, roughness,
emissive color, and tangent-space normal maps to glTF metallic-roughness materials.
Honor authored double-sidedness. Use display color/opacity, then a neutral material,
when no compatible material is available, and report unsupported shader fallbacks.

Support direct shader constants and `UsdUVTexture` connections using
`UsdPrimvarReader_float2` and `UsdTransform2d`. Respect the selected UV primvar,
texture output channel, scale/bias, wrap modes, and raw versus sRGB interpretation.
Bake supported UV transforms into the exported UV data. Handle scalar roughness
and metallic textures by packing glTF's required channels. Only compatible
texture-coordinate sets can be combined; unsupported combinations are diagnosed.

Resolve file assets relative to the layer that authored the value through USD's
resolved asset paths. Embed textures into the visual GLB, downscale without changing
aspect ratio, and reuse decoded images. Support Pillow-readable image formats;
OpenEXR, multi-tile UDIM sets, arbitrary shader networks, MDL, MaterialX, and
specular-workflow conversion are outside scope. A single `<UDIM>` tile can be
resolved when the asset has exactly one tile and UVs remain in that tile; multi-tile
assets require a diagnostic rather than selecting an arbitrary tile.

ALab's preview materials use their documented simplified UVs and JPEG textures;
its full ACEScg/OpenEXR production materials are deliberately not the target.
Emissive surfaces affect their rendered appearance, not physically traced indirect
illumination. Transparency and raster lighting need not match a path tracer.

## Collision policy

- **authored:** include supported geometry carrying `UsdPhysics.CollisionAPI`
  whose composed `physics:collisionEnabled` value is true at the snapshot (use
  its schema default when unauthored). Do not infer the API from an ancestor.
  Authored colliders may be hidden and use proxy purpose. Ignore disabled colliders.
- **visual:** use the imported visible geometry as static triangles.
- **auto:** use enabled authored colliders if any exist inside the selected roots;
  otherwise use visual geometry and record that fallback prominently.
- **none:** emit no collision file or mesh entity.

Authored/auto selection is stage-wide within the selected roots; it does not fill
uncollided objects individually when some colliders exist. In authored mode, no
enabled supported colliders is an error. Auto must not silently fall back if
colliders exist but cannot be converted.

All exported bodies are static. Rigid-body properties, joints, mass, and collider
approximation hints do not create dynamics or convex shapes. Record this static
conversion and any unsupported collider geometry. The collision GLB carries no
textures. Generated mesh entities are double-sided by default.

## Lighting and camera settings

Support sphere lights as point lights and sphere lights with shaping cones as
spot lights. Approximate rect, disk, and cylinder lights by point/spot emitters at
their composed positions, preserving color and direction where applicable and
reporting loss of area-light behavior. Use
`intensity * 2**exposure * light_scale` as the intensity proxy, interpreted as
lumens for local lights and lux for a distant light. Apply enabled color temperature
to the authored light color. Do not claim exact photometric equivalence: finite
emitter size, USD normalization, cone softness, and shaping focus are not preserved.
Record their authored values and the approximation in diagnostics; conversion
requires user adjustment of light scale/exposure for faithful brightness.

Map one distant light to the existing `<sun>` with its transformed direction;
choose the strongest effective distant light deterministically if there are
several and diagnose the omitted lights. Dome textures, light/shadow linking,
light filters, and renderer-specific lighting settings are not translated.

If no usable lights are imported, add clearly labeled default sun/sky illumination
to make the generated experiment inspectable. Never present these defaults as
authored lighting. Use a bounds-based camera for general scenes. The ALab guide
supplies a verified interior camera pose and exposure after the acceptance run;
it must not claim that a bounds-based camera is automatically robot-safe.

## Output bundle and error handling

Write `<out>/<name>/` containing:

- `<name>.glb`: visual scene with embedded textures.
- `<name>.collision.glb`: only when collision geometry is selected.
- `<name>.lights.xml`: converted point/spot entries.
- `<name>.argos`: standalone experiment with Jolt gravity, the selected collision
  mesh, photorealism scenery/lights/exposure, a debug camera, and the Filament viewer.
- `<name>.json`: source identity, options, dependency versions, snapshot, effective
  metadata, bounds, primitive/instance/material/triangle counts, collision policy,
  lighting mappings, and structured diagnostics.

Size the arena from both visual and collision bounds with a two-meter margin and
nonzero dimensions. Do not add a synthetic physics floor or rendering plane that
could intersect the source environment. Set `draw_floor="false"`. Include no
robot/controller dependency in the generated experiment; document how to add
robots and photorealistic sensors to medium `pr`.

All generated asset references are relative to the experiment directory. XML must
be generated with proper escaping; USD names must not be interpolated unescaped
into attributes or comments. Traversal and names must produce deterministic
output ordering and collision-free node IDs derived from source paths.

Fail clearly on missing dependencies, unreadable stages, missing required layers,
invalid numeric options, invalid roots, unusable geometry, and export failures.
Unsupported features and missing textures produce source-path warnings plus JSON
diagnostics; strict mode makes them fatal. Composition failures must not be
reported as a successful complete import. Write to a temporary sibling directory
and publish the bundle only after successful validation; preserve an existing
bundle on failure. Detect format size limits and fail with an actionable message,
rather than emitting a truncated GLB.

## Public acceptance target: ALab

Use the DPEL ALab **v2.3.0 asset structure** with the separately published
**v2.2.0 Techvar Assets** package, the combination linked by the current upstream
release documentation:

- Overview and license link: https://dpel.aswf.io/alab/
- Structure: https://github.com/DigitalProductionExampleLibrary/ALab/archive/refs/tags/v2.3.0.zip
- Geometry, lights, shaders, and 1K textures: https://dpel-assets.aswf.io/usd-alab/alab-techvars.v2.2.0.zip
- Documentation: https://github.com/DigitalProductionExampleLibrary/ALab/blob/v2.3.0/docs/src/pages/documentation.md
- License: https://dpel.aswf.io/alab/alab-license

The Techvar archive is 9,609,335,555 bytes (about 9.6 GB decimal) before extraction;
allow additional space for source extraction, decoded textures, GLBs, and Jolt
cooking. Do not download it during normal builds/tests or without explicit opt-in.
Use the published versioned URLs and record archive SHA-256 digests in acceptance
results after download. Verify package layout before publishing extraction steps.

Target the static set `/root/alab_set01` and the main rig `/root/lightrig01` from
`entry.usda` at frame 1004, excluding animated characters. Validate those paths
against the downloaded release. Prefer render-purpose geometry with preview
material bindings; document proxy-purpose conversion as a faster alternative.
Area lights and subdivision control cages remain documented approximations, not
promises of production-render fidelity. Do not require procedural/fur, 4K EXR,
additional camera, or splat packs.

Create a sample guide under `environments/alab/` with versioned download and merge
instructions, source/license notices, importer commands, diagnostics expectations,
and verified interactive/headless camera settings. Keep upstream sources and
large generated assets untracked. Preserve the applicable upstream notices and
include a description of converted/modified output. Published rendered images
must carry the license-required copyright notice. The ASWF asset license permits
education, research, development, benchmarking, and demonstrations; it is not an
unrestricted commercial-content license.

## Testing and acceptance criteria

### Automated, offline

Create synthetic USDA fixtures and temporary textures independent of ALab:

1. Nested transforms, reset stacks, Y/Z up, units, nonuniform/reflected scales,
   and identical visual/collision placement.
2. Concave faces, holes, orientation, invalid/degenerate topology, indexed UVs,
   normals, material subsets, and analytic primitives.
3. Referenced layers and texture assets relative to their authoring layer;
   native instances and point instancers without prototype duplication.
4. Preview/universal bindings, requested primvar selection, texture channels,
   opacity, UV transforms, embedded texture resolution, and shader fallbacks.
5. Visible/hidden objects, render/proxy purpose, enabled/disabled colliders,
   all collision modes, and no synthetic floor.
6. Transformed point/spot/distant lights, area approximations, missing lights,
   and exposure/scale settings.
7. CLI errors, strict diagnostics, safe names, escaped XML, interrupted exports,
   overwrite protection, and operation from another working directory.

Reload exported GLBs and parse XML/JSON to verify geometry, materials, bounds,
relative references, and counts rather than checking only file existence. Tests
run from the isolated Python environment without Filament or Jolt. Add an optional
headless ARGoS smoke test when both plugins are available.

### Full-scene acceptance, opt-in

Import the ALab static set and lighting rig from the actual downloaded packages.
Record commands, versions, source digests, runtime, output sizes/counts, and all
warnings. Verify generated files reload and the experiment initializes Jolt and
photorealism. Render a headless frame and inspect an interior viewpoint in the
Filament viewer for coherent scale, placement, textures, and usable illumination.
Verify representative floor/wall geometry exists in the collision GLB and perform
a Jolt ray-hit or robot contact check on a selected interior floor. Document a
verified starting pose for users adding robots.

Completion requires this full-scene run as well as offline tests. If download,
plugin availability, or resources block the run, report the target as unvalidated;
passing small fixtures is not evidence that ALab works.

## Explicit exclusions

No runtime USD loading; USDZ; remote/custom asset resolution; USD dynamics, joints,
robot mapping, skeleton evaluation, ongoing animation, semantic labels, arbitrary
shader graphs, MDL, MaterialX, EXR/ACEScg production shading, multi-tile UDIM baking,
subdivision evaluation, displacement, curves, volumes, procedural/fur evaluation,
path-traced lighting equivalence, dome-to-KTX conversion, or unrelated renderer and
Fuel importer refactoring.
