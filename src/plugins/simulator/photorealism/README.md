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

Foot-bot and drone entities render with built-in placeholder bodies
(correct overall dimensions, taken from the qt-opengl draw code).
LEDs are robot-agnostic: any entity with an (optionally directional)
LED-equipped component gets a small emissive cube per LED, positioned
from the LED's anchor and offset and colored from the LED state every
tick, so LED signaling is visible in robot camera images. The floor
entity's colors (any source, including loop functions) are sampled
into a texture and refreshed whenever the floor reports a change.

Segmentation class ids: 0 none, 1 floor, 2 box, 3 cylinder,
4 foot-bot, 5 drone (see `EPRClass` in `render_core/pr_id_scene.h`).

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
- Next: glTF asset registry (replaces placeholder bodies and the
  photorealism -> foot-bot/drone link dependencies), HDR environment
  lighting (extends randomization with IBL swapping), interactive
  viewer.

Known issue: when an ARGoS exception unwinds through teardown while
Filament has in-flight work, the statically linked libc++abi may
abort during unwinding (harmless for passing runs; failing runs
already report their error first).
