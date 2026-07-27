# Official kRPC Tutorial Map

The official documentation is authoritative for KSP behavior. These are
implementation notes, not replacements for the linked tutorials.

| Topic | Official source | Project mapping |
| --- | --- | --- |
| Launch into orbit | [Launch into Orbit](https://krpc.github.io/krpc/0.5.4/tutorials/launch-into-orbit.html) | `src/bin/launch.rs` implements ascent, node planning, orientation, timed burn, and cleanup. |
| Vessel parts | [Interacting with Parts](https://krpc.github.io/krpc/0.5.4/tutorials/parts.html) | Start at `Vessel -> Parts`; see `Part`, `Engine`, `Parachute`, and `DockingPort` in the API reference. |
| Autopilot | [AutoPilot](https://krpc.github.io/krpc/0.5.4/tutorials/autopilot.html) | `begin_vertical_ascent`, `set_target_pitch_and_heading`, and `set_target_direction` wrap the main operations. |
| In-game UI | [User Interface](https://krpc.github.io/krpc/0.5.4/tutorials/user-interface.html) | The local [`UI` reference](api/ui.md) maps the installed generated surface. |

## Launch and circularization

The official example targets a 150 km circular Kerbin orbit and proceeds in
five stages: create telemetry reads and configure autopilot; climb while
pitching toward the horizon; coast after reaching the target apoapsis; calculate
and plan a circularization maneuver; then orient and execute the burn.

The corresponding project executable is intentionally more defensive:

- It begins a gravity turn after `--vertical-ascent-altitude-m` and limits
  acceleration with `--max-acceleration-g`.
- It waits to plan close to apoapsis because atmospheric drag can still change
  the orbit while coasting.
- It uses the vis-viva equation to compute circularization delta-v and the
  Tsiolkovsky rocket equation to estimate burn duration.
- It centers the burn on the maneuver node, reduces throttle for the final
  delta-v, and removes the node afterward.

For the equations and variable meanings, use the comments and helper functions
in `src/lib.rs`; all distances are meters, speeds and delta-v are meters per
second, mass is kilograms, thrust is newtons, and time is seconds.

## Parts

Parts are reached from the active vessel's `Parts` object. The official
tutorial demonstrates three useful patterns:

- Iterate parachutes and deploy each one when an emergency recovery policy says
  it is safe.
- Select a docking port by part identity or criteria, then set it as the
  vessel's control point before docking.
- Combine the specific impulse of currently active, fueled engines using the
  official multiple-engine calculation, rather than treating an arbitrary
  engine's Isp as the vessel Isp.

Parts are stateful game objects. A script must check KSP state and the
part/module type before changing it; deploying parachutes or changing the
control point blindly can make a vessel unrecoverable.

## Autopilot

kRPC's autopilot operates in the selected reference frame. Its primary input is
target pitch and heading, with an optional roll, or a target direction vector.
Set the reference frame before engaging it. Pitch is degrees above the horizon;
heading is compass degrees. The project uses the vessel surface reference frame
for ascent so "up" and headings have the expected launch-site meaning.

The official documentation also explains its PID tuning. Leave the default
stopping time, deceleration time, attenuation angle, time-to-peak, and
overshoot settings unless a real vessel demonstrates a control problem. A
smaller deceleration time turns more aggressively but can overshoot; the
stopping time should stay smaller than the deceleration time.

## UI

The official UI flow is: obtain the stock canvas, add a panel, position its
rect transform relative to the screen, add text and controls, and poll or
stream a button property while updating displayed telemetry. This is useful
design guidance. This project now generates an `services::ui` module from the
locally installed UI schema. Look up the exact available calls in the
[local UI reference](api/ui.md) before adding code; the official tutorial is
not a Rust API signature reference.
