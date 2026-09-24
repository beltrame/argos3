# Jolt mesh entity tests

Verification suite for the `<mesh>` entity of the Jolt plugin: a static
triangle mesh loaded from a glTF 2.0 asset, which robots collide with and ray
casts hit.

The suite covers the four things the entity has to get right, and one property
of the engine it depends on:

- the geometry that reaches Jolt is the geometry in the file, including node
  transforms, the glTF Y-up to ARGoS Z-up conversion, uniform scaling and the
  shape cache;
- ray casts return the distance to the surface, not to a bounding volume, and
  return no hit through an opening;
- robots are stopped and supported by the triangles, including sloped ones,
  and are not deflected while sliding over them;
- the `double_sided` attribute decides whether the winding of the asset
  matters for collision, while ray casts stay two-sided either way;
- a scenario replayed twice gives bitwise identical poses.

The assets are not stored in the repository. `make_meshes.py` writes them into
the build directory when the tests are built; it uses only the Python standard
library and is invoked by CMake. If no Python 3 interpreter is found, the
whole directory is skipped with a status message.

## Running

The tests are declared whenever the Jolt plugin is built, so

    cmake -DARGOS_BUILD_JOLT=ON <path to argos3/src>
    make
    ctest -R jolt_mesh

is all that is needed. `ctest -R jolt_mesh -V` additionally prints every
measured distance, pose and timing, which is where the numbers below come
from. Every test runs headless, single-threaded, in the build directory.

## Tests

| test | what it verifies | tolerance |
|---|---|---|
| `jolt_mesh_rays` | 15 ray checks against the corridor asset, each against a closed-form value: flat floor, ceiling, walls, an oblique beam, three beams meeting the 1:4 ramp, the landing, the end cap, two beams that leave through an opening and one that enters through it. The ramp and the landing sit in a child glTF node with a translation, so a reader that ignored node transforms would answer several of the checks wrongly. | 1e-3 m per ray |
| `jolt_mesh_cache` | one asset declared by six entities: three plain copies, one at `scale="2.0"`, one at `y_up="false"`, one at `double_sided="false"`. Twelve rays check that every instance answers with the geometry its attributes imply, so that sharing a cooked shape between entities, wrapping it in a scaled shape and keying the cache on `y_up` and `double_sided` are all correct. | 1e-3 m per ray |
| `jolt_mesh_collision` | a foot-bot driven at 0.5 m/s into the end cap of the corridor stops one body radius short of it, at 30 - 0.085037 m, and is carried by the mesh landing at z = 2.5 the whole way. The arena has no floor plugin, so the support comes from the mesh alone. | 2e-3 m on X and Z, 5e-2 m of lateral deviation |
| `jolt_mesh_slide` | a foot-bot driven straight for 20 m across a floor tessellated into 2 m cells crosses about 20 internal edges of the triangulation without being pushed sideways. This guards the active-edge information of the cooked shape: a body sliding over an edge Jolt considers inactive has its contact normal replaced by the normal of the face, and over an active edge keeps the edge normal, which points off the surface. | 0.1 m of lateral deviation, 2e-2 m on X and Z |
| `jolt_mesh_ramp` | a foot-bot released 0.1 m above the 1:4 ramp comes to rest on the sloped surface rather than falling through it. Its Jolt body is a cylinder that cannot pitch, so the flat bottom face touches the slope at its uphill edge and the origin anchor sits `slope * radius` above the surface height under its centre; the measured z is compared with that. | 2e-3 m on Z |
| `jolt_mesh_bunker_attitude`, `jolt_mesh_scout_mini_attitude`, `jolt_mesh_spot_attitude` | Real ground-robot bodies drive and turn uphill while their body-up direction follows the ramp normal. | 0.08 rad normal error; at least 0.15 m rise and 0.1 rad yaw |
| `jolt_mesh_traction` | An airborne Bunker receives forward wheel commands for one second but has no supporting contact. It must not translate horizontally; the old chassis-velocity overwrite moved it 0.585 m. | 1e-3 m on X |
| `jolt_mesh_winding` | the same wrongly wound wall across the corridor, run twice from one template. With `double_sided="true"` (the default) the robot is stopped at the wall; with `double_sided="false"` it passes through and is stopped only by the correctly wound end cap 3 m further on. The four ray checks are identical in both runs, which is the point: sensors report the wall either way, so a wrongly wound asset fails silently without the default. | 2e-3 m on X and Z, 1e-3 m per ray |
| `jolt_mesh_determinism` | the collision scenario run twice, dumping the final pose with `%a` and comparing the files with `cmp`. | bitwise |
| `jolt_mesh_bench` | three ray distances against a 105,208-triangle terrain, and the cook time and mean time per ray for 144,000 rays. Only the ray distances are gated; the timings are printed for the record and depend on the machine. | 1e-3 m per ray |

The tolerances are one to three orders of magnitude above the errors measured
below and orders of magnitude below any real failure: a lost node transform,
a missing axis conversion or a robot passing through a wall move the same
quantities by metres. The ray tolerance is also comfortably above the floor
set by Jolt's vertex quantisation, which stores mesh vertices at 21 bits per
component relative to the mesh bounding box, i.e. 1.9e-5 m over the 40 m
corridor and 6.2e-5 m over the 130 m terrain.

## Contact-drive safety regressions

**Operator-approved scope:** "real wheeled and tracked robots can climb walls
and tip over, so keep wheel/track wall traction in ARGoS; only fix clear
unrealism." This explicitly supersedes the original no-climbing requirement
for Bunker and Scout Mini. Their traction is unchanged; the SwarmDeck footprint
critic and tilt guard provide operational tip prevention.

`jolt_mesh_<robot>_wall_push_<speed>_<yaw>_<plain|lip>` drives Bunker,
Scout Mini and Spot at 10/60 cm/s, head-on/45 degrees, for 20 seconds against
a vertical mesh wall, with or without a 3 cm toe. Each run reports peak
absolute pitch/roll (degrees), origin rise (metres) and origin displacement
speed sampled at each physics substep. Spot must stay within 10 degrees,
3 cm rise and 1.5 m/s speed.
Additional Spot cases exercise a 10 cm toe at both speeds and a flank starting
1 cm from the wall with (v, w) = (0.2 m/s, -0.5 rad/s).

At 0.1 m/s, wheel/track plain-wall cases assert 10 degrees / 3 cm limits.
Bunker's 3 cm lip cases assert 10 degrees / 5 cm; Scout's assert 25 degrees /
10 cm, preserving its measured 18.71-degree, 8.87 cm climb. Wheel/track
high-speed and lip cases carry CTest labels `characterization;wheel-track-wall`
(`ctest -L characterization -V`). These tests **do not prohibit climbing or
tipping** outside the bounded low-speed envelopes: the single-box proxies
approximate exposed wheels/tracks, not a safety controller or calibrated
hardware.

`jolt_mesh_<robot>_no_friction` requires no motion under a forward command
on a frictionless surface. Ordinary ground drive remains friction-limited
Jolt surface velocity. The separately gated Spot leg actuator described below
is active only for a checked step with static support within leg reach.

`jolt_mesh_spot_support_contacts` checks the actual Spot callback on a
translated, rotated body. Sole and lower-leg edge support drive; torso, roof,
reversed normals, empty and mixed manifolds do not. Spot's single box includes
leg volume: its bottom 12 cm approximates feet and lower legs. Normals retain
the positive body-up criterion, because disabling steep contacts pins the
solid leg-volume proxy against small rocks on SubT. For normals more than
40 degrees from **world up**, only the drive direction changes: linear Z is
zero and angular surface motion is yaw-only. This preserves contact traction
but prevents a pitched chassis from motoring vertically along a wall. World
axes keep the wall classification stable as the body tips; 16/18-degree ramp
support keeps the original full surface velocity. This is not an articulated
gait model. Bunker and Scout retain their existing contact classification.

In a head-on 0.6 m/s lip test, unfiltered Spot reached 90 degrees pitch and
0.583 m origin rise. Lower-leg-only Spot stays upright; Bunker and Scout still
reach 90 degrees pitch. The existing incline tests continue to exercise
terrain following and differential yaw with all three platforms. Additional
Spot ramp16/ramp18 tests require at least 2.5 m travel in 10 s at 0.3 m/s,
and final X = 3.85 +/- 0.25 m from X = 1 m: downhill travel cannot qualify.

For optional, local SubT assets, configure with
`-DARGOS_JOLT_SUBT_MESH=/path/to/finals_prize_round_world_01.collision.glb`.
Four recorded-route starts must retain their measured progress in 10 s:
2.95/2.94/2.94/2.86 m for flat/approach/entry/pinned (about 2 cm margin). Three
constant-command replays at (11.6, -20), yaw -151 degrees, v=0.3 m/s and
w=0.4/0.8/1.2 rad/s must stay below 1.5 m/s sampled displacement speed.
The recorded ROS base_link z=0.54 is **not** ARGoS origin height: subtract
Spot's 0.50 m base_link offset to get z=0.04. This is a local approximate
replay, not a reconstruction of the entire recorded command history.

The native Spot 10/20/30/35 cm step tests are now **default tests**; no downstream
helper or opt-in step flag is needed. They require completion within 10 s at
0.3 m/s, <1.5 m/s peak speed, <=0.5 m/s upward speed (0.1 mm/s numerical
tolerance), the balance cone, >=2.5 m travel and the correct final height.
Reset while lifting must discard the old target and pass a second climb;
a 40 cm step and a low ceiling must prevent assistance. Physics qualification
must run on an authorized simulation host, not an operator workstation.

## Active Spot balance

Spot has an angular-only Jolt SixDOF constraint: a 30-degree circular swing
cone relative to world up, free yaw and all three translations free. There is
no point anchor, spring motor or position reset; unsupported bodies still
fall under gravity. This models the legged platform's active balance rather
than a freely overturning box. Constraint warm-start impulses reset with the
model, and the constraint is removed before its body is destroyed.

Motion checks sample each physics substep, including instantaneous origin
velocity, finite-difference pose speed (to catch teleports), and total body-up
tilt, not just 10 Hz controller poses. Cone checks
allow 0.1 degree of numerical solver tolerance. An angular-impulse case checks
balance and free yaw before/after Reset. A ledge departure and a separate 1 m
free release check ballistic COM motion and <=0.1 m rebound. Falling speed is
not artificially clamped: only horizontal speed is limited to 1.5 m/s in these
drop tests. Ramp fixtures now start parallel to the slope instead of dropping
horizontally onto it, so their speed limit measures traversal, not a setup
impact. Wheel/track models are unchanged.

## Smooth Spot stepping

`jolt_spot_step.cpp` probes at control rate for a steep obstruction, overhead
clearance, a full-body raised advance and a static, walkable landing under
the future body centre. Ordinary ramp faces are not steps. The full, unshrunken
collision shape is used. A checked step begins a per-body Lift/Advance state:
raise at <=0.5 m/s, then advance at commanded speed while holding the checked
height. This is an idealized leg actuator, not an articulated gait or a
force-calibrated controller. Jolt integrates every pose and resolves contacts;
no helper calls SetPosition, moves an anchor, or disables gravity.

Static support must remain within 35 cm leg reach beneath the body centre at
every physics substep. Zero translation pauses at the current height without
losing the original lift target. Supported yaw-only holds command world-Z yaw
(up to the normal 1.2 rad/s envelope), preserving roll/pitch angular motion.
Reversal, missing support or timeout ends assistance; the timeout continues
while paused. All state resets with the model. Three interrupt regressions hold
for 2 s during advance (zero/yaw-only) or lift, then finish the 35 cm climb.
They bound held height drift to 2 cm, planar drift to 1 mm, tilt to 30.1 degrees
and speed to 1.5 m/s, and require actual yaw in the turning variant. Clearance is
conservative (the initial overhead probe uses the maximum supported step).
The old SwarmDeck pose-jump helper must exclude Spot; wheel/track behavior and
helper limits are unchanged. SwarmDeck's Spot max_step_height remains 0.30 m;
raising it to 0.35 m is a separate planner/configuration decision, needed if
navigation should use the newly qualified full height.

## Measured results

Fix round 2, native fork on tuf (Ubuntu 22.04, GCC 11, Jolt 5.2, Release,
headless Docker capped at 8 CPUs): 73/73 default tests (2.68 s), or 80/80
with the local SubT asset (20.27 s), including seven default step checks and
three balance/drop checks.

| step height (cm) | fully supported by (s) | peak total/up speed (m/s) | final X (m) |
|---|---:|---:|---:|
| 10 | 5.77 | 0.50 / 0.50 | 2.832 |
| 20 | 6.02 | 0.50 / 0.50 | 2.757 |
| 30 | 6.27 | 0.50 / 0.50 | 2.682 |
| 35 | 6.40 | 0.50 / 0.50 | 2.643 |

All four finish at the requested step height with <0.001 degree pitch/roll.
Angular impulse/reset peaks at 30.062 degrees body-up tilt; the ledge drop
peaks at 30.090 degrees (30 degree nominal cone, 0.1 degree solver tolerance).
Free release falls for 0.450 s versus 0.455 s analytically; the supported
rotation off a ledge leaves a shorter detached flight, 0.140 s versus 0.142 s.
Both have zero rebound. Freefall legitimately reaches 4.415 m/s vertically.
Ramp16/ramp18 finish at X=3.850/3.820 m at 0.300 m/s. SubT route travel is
unchanged from balance-only: 2.970/2.963/2.964/2.883 m at physics-substep sampling.
Removing reset cleanup or doubling the lift-speed cap fails the corresponding
new regression. Without assistance, all four step heights stall at X=0.450 m.

### Historical fix round 1

Fix round 1, native fork on tuf (Ubuntu 22.04, GCC 11, Jolt 5.2, Release,
headless Docker capped at 8 CPUs):

    Default: 100% tests passed, 0 tests failed out of 63 (3.26 sec)
    With SubT asset: 100% tests passed, 0 tests failed out of 70 (37.26 sec)

The 12 wheel/track characterization cases remain labelled separately. The
three Spot stress cases are default regressions, not opt-in qualifications.
The 0.6 m/s, 10 cm toe tipped Spot before the world-horizontal drive fix:
89.39 degrees pitch and 0.592 m rise.

| Spot case | peak pitch (deg) | peak roll (deg) | rise (m) |
|---|---:|---:|---:|
| 10 cm toe, 0.1 m/s | 0.09764 | 0.00229 | 0.000905 |
| 10 cm toe, 0.6 m/s | 0.11516 | 0.02107 | 0.001022 |
| flank, (0.2 m/s, -0.5 rad/s) | 0.00830 | 0.00724 | 0 |

The rough pinned SubT start retains 2.87512 m travel versus 2.87395 m before
projection. The 16/18-degree ramps finish at X = 3.87063 / 3.84095 m. Their
reversed-command negative controls fail the X bound despite sufficient path
length. A 1 mm rise-budget negative control and high-speed substitutions into
the low-speed wheel/track cases also fail as intended.

These passing counts exclude the three opt-in step-helper qualifications.
Without that helper, the unchanged box stalls after about 0.45 m on all
10/20/30 cm steps. With the current helper and new projection, the 10 cm step
is climbed but reaches 1.50272 m/s; 20/30 cm steps tip, reaching 2.17866 /
3.03704 m/s. The helper also violates the native Scout low-speed lip envelope
and both new Spot 10 cm toe rise limits (six failures out of 73 tests with
all qualifications enabled). Fix round 2 supersedes Spot's helper disposition
with native smooth lift; the original collision shape is still unchanged.

### Historical mesh measurements

The ray/throughput tables below are the original mesh-entity measurements,
with their original environment recorded at the end; they are not timings
from the current safety-regression run. Those pre-existing tests still pass.

### Ray distances

`jolt_mesh_rays`, 15 checks, 0 failures, worst error **3.040e-06 m**:

| check | expected (m) | measured (m) | error (m) |
|---|---|---|---|
| `floor_down` | 1.0 | 1.000000 | -2.235e-08 |
| `ceiling_up` | 5.0 | 5.000000 | +7.451e-08 |
| `wall_left` | 2.0 | 2.000000 | -4.470e-08 |
| `wall_right` | 2.0 | 2.000000 | -4.470e-08 |
| `wall_diagonal` | 2.8284271247 | 2.828427 | -9.306e-08 |
| `ramp_frontal` | 14.0 | 14.000003 | +3.040e-06 |
| `ramp_down` | 2.5 | 2.500002 | +1.527e-06 |
| `ramp_oblique` | 2.8284271247 | 2.828428 | +6.520e-07 |
| `landing_down` | 2.5 | 2.500000 | -1.490e-07 |
| `end_cap` | 5.0 | 5.000000 | +7.451e-08 |
| `open_end` | no hit | no hit | — |
| `window_through` | no hit | no hit | — |
| `window_above` | 2.0 | 2.000000 | +1.416e-07 |
| `window_below` | 2.0 | 2.000000 | -4.470e-08 |
| `enter_open_end` | 29.0 | 29.000002 | +2.146e-06 |

The three largest errors are the beams that meet the sloped surface or travel
the length of the corridor, which is where the vertex quantisation shows.

`jolt_mesh_cache`, 12 checks, 0 failures, worst error **8.941e-08 m** on
`scaled_wall`. The six entities produce three cooked shapes, which the log
shows directly, one per distinct (file, `y_up`, `double_sided`) key:

    Cooked mesh "corridor.glb": 40 vertices, 40 triangles (double-sided), 8.030e-05 s
    Cooked mesh "corridor.glb": 40 vertices, 40 triangles (double-sided), 3.078e-05 s
    Cooked mesh "corridor.glb": 40 vertices, 20 triangles,                1.781e-05 s

The three plain copies and the scaled one share the first, the `y_up="false"`
copy cooks the second, and the `double_sided="false"` copy cooks the third
with half the triangles.

`jolt_mesh_winding`, 4 checks in each of the two configurations, 0 failures,
worst error **6.706e-08 m**, and identical in both, as expected.

### Poses

| test | quantity | expected | measured | error | tolerance |
|---|---|---|---|---|---|
| `jolt_mesh_collision` | stop position x | 29.914963 | 29.914968 | 5.49e-06 m | 2e-3 m |
| `jolt_mesh_collision` | height z | 2.5 | 2.500000 | 2.38e-07 m | 2e-3 m |
| `jolt_mesh_collision` | lateral deviation | 0 | 8.77e-04 (largest 2.97e-03) | — | 5e-2 m |
| `jolt_mesh_slide` | lateral deviation | 0 | largest 4.81e-03 m | — | 0.1 m |
| `jolt_mesh_slide` | travel x | 4.95 | 4.950776 | 7.76e-04 m | 2e-2 m |
| `jolt_mesh_ramp` | height on the slope | 0.417071 | 0.417052 | -1.88e-05 m | 2e-3 m |
| `jolt_mesh_winding`, `double_sided="true"` | stop position x | 26.914963 | 26.914968 | 5.49e-06 m | 2e-3 m |
| `jolt_mesh_winding`, `double_sided="false"` | stop position x | 29.914963 | 29.914967 | 3.58e-06 m | 2e-3 m |

The two `jolt_mesh_winding` rows are the whole argument for the default: the
same asset, the same robot, the same four ray hits, and 3 m of difference in
where the robot ends up.

`jolt_mesh_determinism`: the two runs write byte-identical pose files, `cmp`
returns 0.

### Throughput

`jolt_mesh_bench`, `terrain.glb`, 105,208 triangles in the file and 210,416
cooked because `double_sided` defaults to true:

| quantity | measured |
|---|---|
| cook time | 0.0911 s |
| rays timed | 144,000 |
| fraction hitting | 0.739 |
| mean time per ray | 0.5871 µs |
| rays per second | 1.70 M |

The rays are cast through `GetClosestEmbodiedEntityIntersectedByRay`, i.e. the
same entry point the proximity and camera sensors use, so the figure includes
the ARGoS side of a query and not only Jolt's.

The same scenario with `double_sided="false"`, which is worth setting on an
asset whose winding is known to be consistent, cooks 105,208 triangles into
one mesh shape in 0.0486 s and answers the same 144,000 rays at 0.3260 µs/ray,
i.e. 3.07 M rays per second. A query against the default descends into both
sub-shapes of the compound, which is where the difference comes from. That
configuration is exercised for correctness by `jolt_mesh_cache` and
`jolt_mesh_winding`; the timing is recorded here only for reference.

### Environment

| | |
|---|---|
| CPU | 13th Gen Intel Core i5-13600K |
| container | Ubuntu 22.04.5 LTS |
| compiler | gcc 11.4.0 |
| CMake | 3.22.1 |
| Python | 3.10.12 |
| ARGoS | commit 4a44b8c plus this branch |
| build | `CMAKE_BUILD_TYPE=Release`, `ARGOS_BUILD_JOLT=ON` |
| engine | `threads="1"`, `<system threads="0"/>`, no visualization |
