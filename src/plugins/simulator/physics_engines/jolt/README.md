# Jolt physics engine plugin

A 3D dynamics physics engine based on
[Jolt Physics](https://github.com/jrouwe/JoltPhysics) (MIT license,
the engine used by Godot 4 and many commercial games), following the
same architecture as the dynamics3d (Bullet) plugin.

## Building

The plugin is disabled by default because Jolt is downloaded (pinned
tag v5.2.0) and built at configure time, which requires network
access:

    cmake -DARGOS_BUILD_JOLT=ON ../src

Jolt is built with `CROSS_PLATFORM_DETERMINISTIC=ON` and statically
linked into the plugin.

## Usage

```xml
<physics_engines>
  <jolt id="jolt" iterations="10" threads="1">
    <floor height="0" friction="1.0" />
    <gravity g="9.81" />
  </jolt>
</physics_engines>
```

With `threads="1"` (the default) stepping is single-threaded and the
simulation is bitwise deterministic across runs (verified by the
`jolt_determinism` test). `iterations` sets the physics sub-steps per
simulation tick; keep the sub-step at or below 0.01 s. Without the
`<gravity>` plugin bodies float, mirroring dynamics3d.

Run `argos3 -q jolt` for the full configuration reference.

The engine works in the ARGoS coordinate system directly (z up); no
axis conversion is needed. Ray casts (`CheckIntersectionWithRay`) are
served by Jolt's narrow-phase query, so ray-based sensors work.

## Supported entities

- **box**, **cylinder**: static or dynamic bodies (`movable`, `mass`
  attributes), shapes shared through `CJoltShapeManager`.
- **foot-bot**: a dynamic cylinder with differential-drive kinematics.
  Wheel velocities are applied as body velocities like dynamics2d, so
  existing controllers behave the same; rotation is locked to the
  z axis (the robot cannot tip) and gravity keeps it on the floor.
  Turret dynamics and gripping are not simulated (the turret and
  perspective-camera anchors are updated kinematically).
- **drone**: a dynamic body driven by the cascaded-PID flight
  controller ported from `pointmass3d_drone_model.cpp` (same gains,
  limits, mass, and inertia). Thrust and torques are applied to the
  body and Jolt integrates the rigid-body dynamics, so the drone can
  collide with and rest on things. The MEMS sensor noise of the
  pointmass3d model is not simulated. Note that the drone responds
  faster than in pointmass3d: the original's trapezoid integrator
  effectively halves accelerations, while Jolt integrates the same
  commands exactly.

New robot models subclass `CJoltSingleBodyObjectModel` (or
`CJoltModel` for multi-body robots) and register with
`REGISTER_STANDARD_JOLT_OPERATIONS_ON_ENTITY`.

## Implementation notes

- ARGoS's `general.h` defines `Log`/`Sqrt`/`Exp`/`Mod` as macros that
  break Jolt's headers; always include Jolt through `jolt_common.h`,
  which suspends them.
- Jolt shape settings passed to compound shapes (for example
  `RotatedTranslatedShapeSettings`) are reference-counted: never pass
  a stack-allocated settings object, create the inner shape first.
- Jolt's default 5 cm convex radius is too large for robot-scale
  geometry; `CJoltShapeManager` shrinks it for small shapes.
- The contact-constraint buffer is allocated from the temp allocator
  at every update: its size (10240 constraints, 32 MB scratch) bounds
  the number of simultaneous contacts, not the number of bodies
  (`max_bodies`, default 16384).

Tests live in `src/testing/jolt/`: box-stack settling, bitwise
determinism, foot-bot differential drive + ray casts, and drone
takeoff/velocity-limit/hover.
