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
- Keep each change focused and validate it with the narrowest relevant command available. Once Cargo tooling exists, use the commands documented in the README and/or this file rather than guessing.
- Use `git` to inspect status, history, and diffs, and use `gh` for GitHub repository, issue, pull-request, and workflow tasks when appropriate. Do not discard user changes or rewrite history unless explicitly requested.
- Tools supplied in the agent context are available in addition to the tools named here. Use the most suitable available tool, and document stable, project-specific tools below when they are introduced.

## Keep project guidance current

- Treat `README.md`, this file, and any future documentation as living project documentation. Update the relevant documentation in the same change when behavior, setup, commands, architecture, dependencies, conventions, or required external configuration changes.
- Update this file whenever its project description, teaching guidance, documented commands, layout notes, validation steps, or tool list becomes inaccurate or incomplete.
- Do not record unverified commands as working instructions. State prerequisites, expected outcomes, and known limitations when they are established.
- Prefer concise documentation that helps the next person or agent work correctly. Remove or revise stale guidance instead of accumulating conflicting instructions.

## Current repository facts

- The repository is newly initialized and currently has no Rust project, automation, tests, or project-specific build commands.
- Planned language: Rust.
- Planned integration: kRPC for Kerbal Space Program.
- Project-specific tools: none documented yet.

When project tooling is added, replace the relevant statements above with verified setup, build, test, lint, run, and simulation/connection instructions.