# kRPC Documentation

This directory is the starting point for agents and contributors working with
kRPC in this repository. Read the focused guide before searching generated code.

## Guides

| Document | Use it for |
| --- | --- |
| [Rust client guide](rust-client.md) | Connecting, using `KspClient`, and making or wrapping a low-level RPC call |
| [Generated API references](api/README.md) | Finding the locally generated service, class, method, parameter, return type, and KSP scene information |
| [Official tutorial map](official-tutorials.md) | Applying the official launch, parts, autopilot, and UI tutorials to this Rust project |

## Authority and version boundaries

- The official kRPC documentation explains KSP behavior and RPC semantics. The
  tutorial map links to the selected official **kRPC 0.5.4** pages.
- `services/KRPC.*.json` is this repository's machine-readable source of truth
  for the available RPC surfaces. Its generated Rust output is
  `src/services/`.
- The Rust transport/client is `krpc-mars`, which is no longer actively
  maintained. Treat its API as an implementation detail: use `KspClient` first,
  then add a small, documented wrapper there when a raw SpaceCenter call is
  needed.
- The current schemas cover SpaceCenter, UI, Drawing, InfernalRobotics,
  KerbalAlarmClock, LiDAR, and RemoteTech. `KRPC.Core.json` and
  `KRPC.DockingCamera.json` remain excluded because their `BYTES` type is
  unsupported by the generator; Core is also unnecessary for the Rust client's
  connection protocol.
- **Decision:** do not add `BYTES` support unless a mission needs it.
  `krpc-mars` already performs connection setup and normal stream management
  without generated Core bindings. The exclusion therefore does not remove
  vessel, flight, parts, maneuver, docking-port, UI, or ordinary telemetry
  control. It only leaves Core's advanced diagnostics, server-side
  expressions/events, and stream-rate tuning unavailable, plus the optional
  DockingCamera image buffer.

## Keeping the reference current

After replacing any `services/KRPC.*.json`, regenerate both the Rust
binding and the Markdown reference from the repository root:

```powershell
cargo build
powershell -ExecutionPolicy Bypass -File docs\generate-api-references.ps1
```

Commit the updated `docs/api/` files with the schema change. The
reference is generated from the local schema rather than copied from the
generated Rust file, so it is compact, searchable, and remains tied to the
actual RPC version used by the project.
