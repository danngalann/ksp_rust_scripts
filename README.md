# KSP Rust kRPC Scripts

Rust programs for controlling and experimenting with Kerbal Space Program through the [kRPC](https://krpc.github.io/krpc/) mod.

This is a learning project. Its programs will cover activities such as launches, landings, orbital maneuvers, and mission automation while building practical understanding of:

- Rust and Cargo
- kRPC and KSP flight control
- Rocket flight and orbital mechanics
- The mathematics behind the maneuvers

## Project structure

- `src/lib.rs`: the `KspClient` library, where low-level kRPC calls are turned into focused rocket operations.
- `src/bin/vessel_info.rs`: a runnable connection test that prints basic data for KSP's active vessel.
- `src/bin/launch.rs`: a conservative launch-to-orbit controller with explicit propulsion staging, max-Q and acceleration limits, maneuver alignment checks, and orbit validation.
- `services/`: compatible `KRPC.*.json` API descriptions copied from the installed kRPC mod, including SpaceCenter, UI, Drawing, and installed mod integrations.
- `build.rs`: generates Rust bindings from the service descriptions during every relevant Cargo build.
- `docs/`: generated per-service API references and focused Rust/kRPC guides for contributors and agents.

The JSON files are kRPC's machine-readable API descriptions, not save data or a configuration to edit by hand. `krpc-mars-terraformer` is a Rust build dependency, unrelated to HashiCorp Terraform: it reads those JSON descriptions and writes the generated `src/services/` bindings. The generated files are ignored by Git because Cargo can recreate them. The installed `KRPC.Core.json` and `KRPC.DockingCamera.json` use a `BYTES` type that the current generator does not support; Core is not needed because `krpc-mars` implements the connection protocol itself. After changing a compatible service schema, run `cargo build` and `powershell -ExecutionPolicy Bypass -File docs\generate-api-references.ps1` to refresh its generated API reference.

`vendor/krpc-mars-terraformer/` is a local GPL-3.0 copy of the upstream generator. It changes its enum-value parser from `u32` to `i32`, allowing current kRPC's legitimate negative bit-flag value, `MapFilterType::All = -1`. It also retains original RPC procedure names while assigning `_overload` suffixes to Rust method names that would otherwise collide after conversion to `snake_case`. Keep this patch until an upstream release supports both the current kRPC schemas and this project can return to the Git dependency.

## Prerequisites

- Rust and Cargo. Verify with `cargo --version`.
- Protocol Buffers' `protoc` compiler, required by the upstream `krpc-mars` crate. Install it from an elevated PowerShell or Command Prompt:

  ```powershell
  winget install --exact --id Google.Protobuf --accept-package-agreements --accept-source-agreements
  ```

  Restart VS Code after installation, then verify it with `protoc --version`.
- Kerbal Space Program with the kRPC mod installed.
- The local kRPC RPC server running at `127.0.0.1:50000`.

This project was set up against the kRPC installation at `S:\GAMES\SteamLibrary\steamapps\common\Kerbal Space Program`. Its `settings.cfg` enables automatic server startup and connection acceptance, with the default RPC port `50000`.

## Build and run

1. Start KSP and load a save into the Flight scene with a vessel selected. Wait for the kRPC server to start.
2. In this repository, compile the project:

	```powershell
	cargo build
	```

3. Run the connection test:

	```powershell
	cargo run --bin vessel_info
	```

Expected output includes the active vessel name, type, situation, mass in kilograms, and mission elapsed time in seconds. A connection-refused error means KSP/kRPC is not running or its RPC port differs from `50000`.

Run the checks with:

```powershell
cargo test
```

Run the known-good Kerbin launch profile with:

```powershell
cargo run --bin launch -- --propulsion-stage-count 2 --target-apoapsis-km 80
```

`--propulsion-stage-count` includes the initial engine-ignition stage. The
launcher requires nonzero maximum thrust from engines assigned to KSP's pending
UI stage, waits up to `--stage-ignition-timeout-s` for actual usable thrust, and never advances
past the configured count. Match this value to the vessel's staging stack
before launch; the default assumes a conventional two-stage rocket.

kRPC exposes KSP's internal stage cursor, which is one greater than the stage
activated by the next Space press. The launcher converts that cursor before
matching engine-part stage numbers; for example, internal stage 4 activates the
stage labeled 3 in the flight UI.

Crafts may include a manually controlled non-propulsive interstage between two
configured propulsion stages. If the player activates that interstage while
the launcher is orienting for or executing the maneuver, the controller notices
the stage-cursor change, verifies that the active vessel has no active engine,
then activates and verifies the next configured propulsion stage at closed
throttle. A throttle loss without either exhausted engines or an observed manual
stage transition still aborts without pressing Space.

For the documented `Small sat` layout, `--propulsion-stage-count 2` counts
stage 3 (launch propulsion) and stage 1 (orbital propulsion). Press Space once
for stage 2 when separation is desired; do not also activate stage 1 manually.
Stage 0 remains outside the configured propulsion budget, protecting the
fairing and satellite deployment sequence from automatic staging.

The default profile launches east at 90 degrees, limits acceleration to 3 g,
and limits Kerbin dynamic pressure to 25,000 Pa with hysteresis. It requires the
target altitude to clear the current body's atmosphere plus the configured
safety margin. Airless bodies skip dynamic-pressure control, but their pitch
schedule and other defaults still need craft-specific testing. Before a maneuver
burn, it also reserves 20 seconds in real time for kRPC autopilot alignment;
raise `--maneuver-orientation-reserve-s` for a slow-turning craft. Use
`cargo run --bin launch -- --help` for all profile, alignment, orbit-tolerance,
and throttle settings.

At the end of a run, the launcher reports one of three results: an approximately
circular target orbit, a safe but off-target orbit, or an unsafe/suborbital
trajectory. Unsafe results return an error. Cargo tests cover pure controller
decisions, but launches must still be tested in a disposable save because no
unit test can establish a vessel's control authority, staging layout, or
structural margin.

Cargo is Rust's equivalent of npm plus its build/test runner: it downloads dependencies, runs `build.rs` to generate the bindings, compiles the crate, and runs the requested command. The first build may take longer while Cargo downloads and compiles the two `krpc-mars` Git dependencies.

## Contribution and learning approach

Code and documentation should favor clarity over cleverness. Changes should explain unfamiliar Rust, Cargo, mathematical, and orbital-mechanics concepts in context, including relevant assumptions and units. See [.github/copilot-instructions.md](.github/copilot-instructions.md) for the repository guidance used by AI agents.