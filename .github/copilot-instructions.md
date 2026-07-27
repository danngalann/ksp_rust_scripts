# KSP Rust kRPC Scripts

## Project purpose

This repository contains Rust scripts and programs for the kRPC mod for Kerbal Space Program. Programs will automate and model flight activities such as launches, landings, orbital maneuvers, and related mission tasks.

The project has two equally important goals:

1. Build enjoyable, working KSP/kRPC programs.
2. Teach the repository owner Rust, Cargo, orbital mechanics, rocket flight, and the mathematics needed to understand the programs.

Treat learning as a product requirement, not an afterthought.

## Teaching style

- Assume the owner is new to Rust and Cargo unless the conversation establishes otherwise.
- Explain each non-obvious Rust or Cargo concept before or alongside using it. When useful, compare it to JavaScript: for example, ownership versus references, `Result` versus thrown errors, modules/crates versus files/packages, and Cargo versus npm.
- Build explanations incrementally. State what a concept does, why it is needed here, and how it affects the code being written.
- Do not hide important logic behind unexplained abstractions or large code dumps. Prefer small, runnable steps and explain how to run and verify each one.
- Teach orbital and mathematical ideas as they become relevant. Define variables, include units, distinguish vectors from scalars, show the governing equation when useful, and connect calculations to an observable KSP behavior.
- Make clear when a result is an in-game approximation, a kRPC limitation, or a simplified physical model. Do not present assumptions as physical facts.
- Favor safe, readable, idiomatic Rust. Explain compiler errors and borrow-checker feedback in plain language, including the underlying rule that triggered them.

## Working in this repository

- Inspect the existing layout and documentation before adding or changing implementation conventions. This repository is intentionally being established in stages; do not assume a Cargo workspace, package name, crate structure, test command, or KSP connection configuration exists until it is added.
- Begin kRPC API work with [`docs/README.md`](../docs/README.md). Read its focused Rust-client, generated API, and official-tutorial guides before searching generated bindings. The local service references are generated from `services/KRPC.*.json`; use them to locate calls instead of grepping `src/services/`.
- The current generated Rust surface includes SpaceCenter, UI, Drawing, InfernalRobotics, KerbalAlarmClock, LiDAR, and RemoteTech. Look up the local reference before using a service. Do not add `KRPC.Core.json` or `KRPC.DockingCamera.json`: their `BYTES` type remains unsupported, and the project intentionally defers support because `krpc-mars` already handles connection setup and normal streams without Core bindings. Revisit that decision only for Core's advanced control-plane APIs or the optional DockingCamera image buffer.
- Keep each change focused and validate it with the narrowest relevant command available. Once Cargo tooling exists, use the commands documented in the README and/or this file rather than guessing.
- Use `git` to inspect status, history, and diffs, and use `gh` for GitHub repository, issue, pull-request, and workflow tasks when appropriate. Do not discard user changes or rewrite history unless explicitly requested.
- When the user requests commits, create small commits with short imperative subjects, grouping files together only when they form a coherent change, and always push each commit after creating it.
- Tools supplied in the agent context are available in addition to the tools named here. Use the most suitable available tool, and document stable, project-specific tools below when they are introduced.

## Keep project guidance current

- Treat `README.md`, this file, and any future documentation as living project documentation. Update the relevant documentation in the same change when behavior, setup, commands, architecture, dependencies, conventions, or required external configuration changes.
- Update this file whenever its project description, teaching guidance, documented commands, layout notes, validation steps, or tool list becomes inaccurate or incomplete.
- Do not record unverified commands as working instructions. State prerequisites, expected outcomes, and known limitations when they are established.
- Prefer concise documentation that helps the next person or agent work correctly. Remove or revise stale guidance instead of accumulating conflicting instructions.

## Current repository facts

- The root Cargo package is `ksp_krpc`, a Rust library that wraps kRPC operations and executable examples in `src/bin/`.
- The `services/KRPC.SpaceCenter.json` definition is stored in `services/`. `build.rs` generates ignored Rust bindings in `src/services/` using the patched local `vendor/krpc-mars-terraformer/`. Do not add the installed `KRPC.Core.json`: its `BYTES` type is unsupported by the current generator and is unnecessary for the library's connection protocol. The vendored generator changes `EnumValue.value` from `u32` to `i32` so current kRPC's `MapFilterType::All = -1` parses correctly, and keeps original RPC procedure names while adding `_overload` only where Rust `snake_case` names collide. Preserve its GPL-3.0 license and replace the patch only after verifying an upstream-compatible release.
- `cargo build` generates bindings and compiles the project. It requires the `protoc` compiler because the upstream `krpc-mars` crate builds Protocol Buffers code. `cargo test` runs the project's checks. `cargo run --bin vessel_info` connects to a running local KSP/kRPC instance and prints the active vessel's basic telemetry.
- The local KSP installation is `S:\GAMES\SteamLibrary\steamapps\common\Kerbal Space Program`; its kRPC configuration currently uses `127.0.0.1:50000` for RPC and `127.0.0.1:50001` for streaming.
- Project-specific tools: Cargo/Rust, Protocol Buffers (`protoc`), kRPC, `krpc-mars`, and `krpc-mars-terraformer`.
- When any compatible `services/KRPC.*.json` changes, run `cargo build` and `powershell -ExecutionPolicy Bypass -File docs\generate-api-references.ps1`, then commit the refreshed `docs/api/` references.

Keep these facts in sync with verified Cargo commands, crate dependencies, generated-code conventions, and kRPC connection settings.