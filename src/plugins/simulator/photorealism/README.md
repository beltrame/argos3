# Photorealism plugin

Photorealistic rendering of the ARGoS space using
[Google Filament](https://github.com/google/filament), designed for
headless per-robot camera sensors (RGB, depth, segmentation) and
sim-to-real transfer. The render engine is owned by the
`<photorealism>` medium, so it works with no `<visualization>` section
and no display server (Vulkan headless; use lavapipe for CPU-only
nodes).

## Building

1. Download and extract the Filament SDK (v1.72.1 or later):

       curl -LO https://github.com/google/filament/releases/download/v1.72.1/filament-v1.72.1-linux.tgz
       mkdir -p ~/filament-sdk && tar xzf filament-v1.72.1-linux.tgz -C ~/filament-sdk

2. On Linux, the prebuilt SDK needs the static libc++ runtime. Either
   install it (`apt install libc++-dev libc++abi-dev`) or extract it
   locally without root:

       mkdir -p ~/filament-sdk/libcxx && cd ~/filament-sdk/libcxx
       apt-get download libc++-20-dev libc++1-20 libc++abi-20-dev libc++abi1-20
       for d in *.deb; do dpkg -x "$d" root/; done

   The build looks for `libc++.a`/`libc++abi.a` next to the SDK
   (`<FILAMENT_DIR>/../libcxx/root/usr/lib/llvm-*/lib`), in
   `FILAMENT_LIBCXX_DIR`, or in the system paths.

3. Configure ARGoS with:

       cmake -DFILAMENT_DIR=~/filament-sdk/filament ../src

   The plugin is skipped (with a status message) when Filament is not
   found.

The prebuilt Filament archives are compiled with clang/libc++ while
ARGoS builds with the system compiler. The plugin's CMakeLists
pre-links Filament and the static libc++ runtime into a single
relocatable object and localizes everything except the Filament public
API ("symbol firewall"), so the two C++ runtimes cannot capture each
other's symbols.

## Usage

```xml
<media>
  <photorealism id="pr" backend="vulkan">
    <sun direction="0.6,0.2,-1" intensity="100000" cast_shadows="true" />
    <skybox color="0.53,0.71,0.92" />
    <debug_camera position="2.5,2.5,2" look_at="0,0,0.2" fov="45"
                  resolution="640,480" period="1" dump="frames" />
  </photorealism>
</media>
```

Run `argos3 -q photorealism` for the full configuration reference.

## HDR environments and scenery

Instead of the constant sky and ambient light, the medium can take
its lighting and sky from an HDR environment prefiltered with
Filament's `cmgen` tool, and static glTF props can dress the scene
(render-only, no physics):

```xml
<photorealism id="pr">
  <environment ibl="assets/venetian_crossroads_ibl.ktx"
               skybox="assets/venetian_crossroads_skybox.ktx"
               intensity="25000" />
  <sun direction="0.5,0.4,-0.8" intensity="70000" cast_shadows="true" />
  <scenery>
    <prop model="assets/DamagedHelmet.glb"
          position="4,0,1.2" orientation="180,0,90" scale="1.5" />
  </scenery>
</photorealism>
```

Generate the KTX pair from any equirectangular HDR with:

    cmgen --format=ktx --size=256 --deploy=. environment.hdr

`cmgen` follows the glTF y-up convention; re-project the HDR to z-up
first (see `rotate_env.py` in the argos3-examples drone_photo_tour
experiment) so the sky ends up overhead in the ARGoS world. The
`<environment>` replaces the ambient light and, when a skybox is
given, the `<skybox>` color; the `<sun>` remains available for crisp
shadows. Props take the same position/orientation/scale attributes as
visual descriptors and appear in the RGB image only (segmentation
id 0), so ground-truth labels stay clean.

On a machine without a GPU, select the software Vulkan driver:

    VK_DRIVER_FILES=/usr/share/vulkan/icd.d/lvp_icd.json argos3 -c experiment.argos

## Camera sensor

Robots obtain images through the generic `photorealistic_camera`
sensor (control interface
`argos3/plugins/robots/generic/control_interface/ci_photorealistic_camera_sensor.h`):

```xml
<sensors>
  <photorealistic_camera implementation="default" medium="pr"
                         anchor="origin" position="0.1,0,0.15"
                         orientation="0,0,0" resolution="64,64"
                         fov="60" near="0.05" far="20"
                         modalities="rgb,depth,seg"
                         framerate_divider="1" />
</sensors>
```

Each frame carries RGB pixels, metric depth along the optical axis
(far-plane value where there is no geometry), and per-pixel
entity/class segmentation ids. The camera looks along the +x axis of
its mount frame with +z up. Frames are pipelined by default (the frame
seen in a control step depicts the previous tick, like a real camera;
the GPU renders while the CPU simulates); set `latency="immediate"`
on the medium for same-tick frames. With `framerate_divider="n"` a
camera renders every n-th tick only, and cameras are skewed
round-robin to spread the load.

Run `argos3 -q photorealistic_camera` for the full reference.

## Interactive viewer

The `filament` visualization opens a window on the medium's scene:

```xml
<visualization>
  <filament medium="pr" resolution="1280,720"
            position="2.5,2.5,2" look_at="0,0,0.25" speed="1" />
</visualization>
```

The experiment behaves exactly as it does headless (the medium owns
the renderer, the scene, and the robot cameras; the window uses its
own Filament renderer, so its vsync never interferes with the sensor
pipeline). SPACE pauses, N single-steps; W/A/S/D/Q/E fly the camera
(SHIFT accelerates), left-drag looks around, right- or middle-drag
pans, and the scroll wheel dollies; ESC quits. `speed` scales real
time (0 = as fast as possible). With `inset_camera="<robot id>"` the
window shows what that robot's photorealistic camera sees as a
bottom-right inset, rendered live at the window frame rate from the
sensor's pose, field of view, and aspect ratio (`inset_size` sets its
height as a window fraction). With `screenshot="<prefix>"` the window
is saved to `<prefix>_<clock>.png` every `screenshot_period` ticks. Building the viewer needs the SDL2 headers
(`apt install libsdl2-dev`, or extract them next to the SDK like the
libc++ runtime; see `FindARGoSSDL2.cmake`). The window is X11 (via
XWayland on Wayland desktops), matching the surface support of the
prebuilt Filament Vulkan backend.

## Domain randomization

For sim-to-real transfer, the medium can redraw the environment from
configured ranges. Randomization uses the ARGoS RNG ("argos"
category), so datasets are reproducible per seed; it is applied at
startup and, unless `on_reset="false"`, at every simulation reset:

```xml
<photorealism id="pr">
  <sun direction="0.6,0.2,-1" intensity="100000" cast_shadows="true" />
  <randomization on_reset="true">
    <sun intensity="60000:140000" elevation="25:80" azimuth="0:360" />
    <sky color_min="0.3,0.4,0.55" color_max="0.7,0.8,1.0" />
    <materials targets="box,cylinder,floor"
               roughness="0.2:1.0" color_jitter="0.3" />
  </randomization>
</photorealism>
```

`<sun>` draws the intensity (lux) and the light direction (elevation
and azimuth, degrees); `<sky>` draws each sky color channel between
the two colors; `<materials>` randomizes roughness and jitters the
authored base colors of the listed visual classes. LED emissives and
segmentation ids are never randomized, so ground-truth labels are
invariant across draws (asserted by the `photorealism_randomization`
test: same seed produces bitwise-identical frames, different seeds
produce different RGB but identical depth and segmentation).

Loop functions can drive randomization directly through the medium:
`SetSunlight()`, `SetSkyColor()`, `SetMaterialParam()`,
`SetMaterialColor()`, and `RandomizeAll()`. A typical dataset
generator combines `on_reset="true"` with
`CLoopFunctions::PostExperiment()` calling `CSimulator::Reset()` to
restart the run under a new random environment while dumping camera
frames. IBL/HDR environment swapping and glTF asset swapping arrive
with the asset pipeline.

## Scaling and performance

Cameras render into private targets that are composited into one
atlas render target per modality by a blit pass, so each tick issues
a single GPU readback per modality regardless of the camera count.
Besides reducing readback overhead, this works around a race in
Filament's Vulkan `readPixels` (both v1.72.1 and current `main`):
its completion handler frees command buffers from a worker thread
while the driver thread may still be allocating from the same
externally synchronized `VkCommandPool`, which segfaults reliably at
~100 readbacks per frame. Keep the readback count low if you ever
touch this code path.

With `stats="true"` on the medium, a timing summary is printed at the
end of the experiment (`src/testing/photorealism/scale_50/` is the
benchmark: 50 foot-bots, each with a 128x128 rgb+depth+seg camera).
On an Intel Iris Xe laptop iGPU (Vulkan, headless) the medium update
costs per tick, all 50 cameras and all modalities:

| Configuration               | avg ms/tick |
|-----------------------------|-------------|
| shadows on                  | ~82         |
| shadows off                 | ~42         |
| `framerate_divider="2"`     | ~38         |

The cost splits roughly half/half between CPU-side render submission
and waiting for the GPU, and scales linearly with camera count,
resolution, and framerate divider. A discrete GPU shortens the wait
side substantially; the submission side scales with single-core CPU
speed.

## Robot visuals

Robot visuals are data-driven: an entity type renders from a glTF
model when a `<type>.visual.xml` descriptor is found in the asset
search path. `<type>` is the string the entity returns from
`GetTypeDescription()`, matched exactly, so a `scout_mini` entity
needs `scout_mini.visual.xml` and not `scout-mini.visual.xml`; the
only sign of a mismatch is one `[INFO] Photorealism: no visual model
for entity type` line, after which that robot is invisible to the
cameras and to the photorealistic lidar (the medium's `asset_path` attribute, then the
`ARGOS_PHOTOREALISM_ASSET_PATH` environment variable, then the
installed assets in `share/argos3/photorealism`):

```xml
<visual>
  <model path="foot-bot.glb" scale="1" position="0,0,0"
         orientation="0,0,90" />
  <segmentation class="4" />
</visual>
```

The model path is relative to the descriptor; 'orientation' (Euler
z,y,x degrees) maps the glTF frame onto the ARGoS frame ("0,0,90"
turns the y-up glTF convention into z-up). Each entity gets two
gltfio instances sharing the model's GPU buffers: one in the main
scene with the glTF PBR materials, one in the segmentation scene with
its materials swapped for the id-encoding material. `assets/` ships
`foot-bot.glb` (procedural, dimensions from the qt-opengl code) and
`drone.glb` (converted from the drone plugin's OBJ models); see
`assets/generate_assets.py` to regenerate or add models.

Without a descriptor, foot-bot and drone entities fall back to
built-in placeholder bodies and boxes/cylinders render procedurally.
LEDs are robot-agnostic either way: any entity with an (optionally
directional) LED-equipped component gets a small emissive cube per
LED, positioned from the LED's anchor and offset and colored from the
LED state every tick, so LED signaling is visible in robot camera
images. The floor entity's colors (any source, including loop
functions) are sampled into a texture and refreshed whenever the
floor reports a change.

Segmentation class ids: 0 none, 1 floor, 2 box, 3 cylinder,
4 foot-bot, 5 drone, 6 scenery, 7 bunker-mini, 8 scout-mini,
9 spot, 10 bunker (see `EPRClass` in `render_core/pr_id_scene.h`);
glTF visuals take their class id from the descriptor.

## Status

- M1 (done): render core, box/cylinder/floor primitives, sun +
  ambient lighting, shadows, headless debug camera dumping PNGs.
- M2 (done): `photorealistic_camera` sensor with RGB/depth/
  segmentation, pipelined async GPU readback, segmentation id scene.
- M3 (done): placeholder foot-bot/drone visuals, generic LED
  emissives, floor texture sync, robot segmentation classes.
- M4 (done): readback atlas (one readPixels per modality per tick),
  50-camera scale test, timing stats (`stats="true"`), working around
  the Filament Vulkan readPixels command-pool race.
- M6 (done): Jolt physics engine plugin
  (`src/plugins/simulator/physics_engines/jolt/`, built with
  `-DARGOS_BUILD_JOLT=ON`); the camera sensors work with any physics
  engine, including Jolt.
- M7 (done): domain randomization (`<randomization>` on the medium +
  loop-function API), deterministic per seed.
- M5 (done): interactive `filament` visualization (SDL2/X11 window,
  free-fly camera, pause/step, real-time pacing).
- glTF asset registry (done): data-driven robot visuals with
  descriptors, instancing, and per-instance segmentation; ships
  foot-bot and drone models and removes the photorealism ->
  foot-bot/drone link dependencies. Also fixed a vertical flip in
  the sensor readback that the flip-invariant tests had not caught
  (an orientation assertion now guards it).
- Next: HDR environment lighting (extends randomization with IBL
  swapping and SetIBL/SwapAsset loop-function calls).

Known issue: when an ARGoS exception unwinds through teardown while
Filament has in-flight work, the statically linked libc++abi may
abort during unwinding (harmless for passing runs; failing runs
already report their error first).
