# Rust Client Guide

## Start with the project wrapper

`KspClient` in `src/lib.rs` owns the `krpc_mars::RPCClient` connection and
offers mission-level methods such as `set_throttle`, `stage`,
`set_target_pitch_and_heading`, `plan_circularization_node`, and
`warp_to`. Prefer it over raw calls because it gives the rest of the project
one readable, testable vocabulary with documented units and safety behavior.

```rust
use std::error::Error;
use ksp_krpc::KspClient;

fn main() -> Result<(), Box<dyn Error>> {
    let mut ksp = KspClient::connect("My KSP program")?;
    let vessel = ksp.active_vessel_info()?;
    println!("Controlling {}", vessel.name);

    ksp.set_throttle(0.0)?;
    Ok(())
}
```

`Result` is Rust's explicit error value, unlike a JavaScript exception. The
`?` operator returns an error to the caller immediately if a connection or RPC
call fails. `&mut self` on most `KspClient` methods means Rust requires
exclusive access while a call uses the underlying mutable RPC connection.

`KspClient::connect` uses `127.0.0.1:50000`; use `connect_to` only when the
local kRPC configuration has a different endpoint. KSP must be in the Flight
scene before calls that are marked `FLIGHT` in the API reference.

## Adding a missing SpaceCenter operation

Do not hand-search the generated binding first. Look up the owning class and
method in [the API references](api/README.md), then inspect the exact
generated signature only if the type mapping is unclear. Generated methods
build a `CallHandle<T>`; `rpc.mk_call(&handle)` sends it and returns `T`.

The project pattern is:

```rust
use services::space_center;

let vessel = self.rpc.mk_call(&space_center::get_active_vessel())?;
let control = self.rpc.mk_call(&vessel.get_control())?;
self.rpc.mk_call(&control.set_throttle(0.5))?;
```

Keep low-level handles local, return ordinary Rust values from a new
`KspClient` method where practical, document units and KSP assumptions, and
propagate errors with `?`. `batch_call!` is appropriate when independent
telemetry reads can be requested together; see `active_vessel_info` for the
existing pattern.

Generated Rust names are normally lowercase snake case (`get_ApoapsisAltitude`
becomes `get_apoapsis_altitude`). The vendored generator adds `_overload` only
when two RPC names would otherwise collide after that conversion. The API
reference shows the schema's original RPC name; the generated binding is final
authority for the Rust spelling.

## Streaming telemetry

For rapidly changing telemetry, use `krpc_mars::StreamClient` instead of
polling an RPC in a tight loop. Connect it from the existing `RPCClient` using
the streaming endpoint (`127.0.0.1:50001` in this installation), convert a
regular call handle with `to_stream()`, send that handle through `mk_call`, and
read updates with `recv_update()`.

Each update returns an `Option` because kRPC does not send a stream value when
it has not changed. Remove a stream with its handle's `remove()` call through
the RPC client when it is no longer needed. Keep both clients alive for the
stream's lifetime. The upstream `krpc-mars` README is the source for the exact
stream-client API; add a focused `KspClient` wrapper before using streams in
mission code.

## Scope limitations

The build script generates the compatible service schemas committed in
`services/`, including `UI`. Use the generated `services::ui` module only after
checking its [local API reference](api/ui.md); its available calls reflect the
installed kRPC version, not a tutorial's examples. `KRPC.Core.json` is
intentionally excluded because its `BYTES` type is unsupported and
`krpc-mars` does not need it for RPC or streaming connections.

When the unmaintained `krpc-mars` client cannot represent a required schema
feature, first capture the exact schema method and type, then isolate any
workaround behind a focused wrapper and document the compatibility reason.
