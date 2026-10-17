# Technical decisions

Decision date: 2026-10-17 (Asia/Seoul). These are implementation boundaries and acceptance intentions. Executed build/test evidence belongs in the reports; this document does not declare unrun checks successful.

| Decision | Reason and consequence |
| --- | --- |
| Build an independent C++20 core first | Targeted inspection found no installed Unreal Editor, Visual Studio C++ toolchain, Windows SDK, CMake or Ninja. Core development can proceed without an engine installation. Unreal rendering and mobile builds remain blocked/unverified until their tools exist. |
| Use a workspace-local portable Zig compiler | Official Windows x86_64 0.17.0 archive selected: 100,268,137 bytes. Avoid a system-wide compiler installation or PATH mutation. Execute a smoke compile, then compile/run the project before claiming build success. |
| Keep the core independent of renderer types | Physics, atmosphere, coordinates, time and instruments use ordinary C++ values. A future engine adapter consumes snapshots and maps units/axes at its boundary. An adapter design is not an Unreal integration test. |
| Use SI internally and explicit coordinate frames | Geodesy uses double precision WGS84/ECEF and local tangent coordinates. Flight body axes are forward/right/down; local navigation is north/east/down. Convert explicitly to ENU or renderer coordinates; avoid silent axis swaps and metre/centimetre mixing. |
| Fixed simulation step, bounded catch-up | Physics time is independent of display rate. Report dropped time rather than inventing physics steps. Reproducibility claims are limited to matching inputs and the same build until cross-platform evidence exists. |
| Synthetic aircraft and ground baseline | Original, openly identified illustrative aerodynamic coefficients and a flat test ground permit deterministic tests. They do not establish real-aircraft fidelity, terrain coverage or validated handling. |
| Providers disabled by default | No runtime maps, weather API, telemetry or paid service is selected. Coordinates and locally generated terrain cannot stand in for licensed real-world imagery. |
| Explicit save/config versions | The first supported format must identify its version, units and frame conventions; validate before mutating live state. Reject unsupported versions and malformed/nonfinite data. Preserve authoritative physical state and any state needed for deterministic continuation. Migration is future work until tested. |

Compiler archive SHA-256:

```text
b5663f69581dcf391293fbf16c06cb80d81d806545ce618b4d0bab7f0eb8c428
```

Archive identity is checked against [official Zig metadata](https://ziglang.org/download/index.json). Compiler/runtime license notices must follow the actual distribution and build target, as described in [LICENSES.md](LICENSES.md).

Unreal remains the future renderer candidate because it matches the requested direction. Hardware feature support, editor startup, engine/plugin version compatibility, frame timing and packaged performance must be measured after installation. No Lumen, Nanite, ray-tracing, VRAM or mobile capability is asserted from hardware names alone. Match the selected Unreal version to its [official hardware/software requirements](https://dev.epicgames.com/documentation/en-us/unreal-engine/hardware-and-software-specifications-for-unreal-engine); current online requirements are not evidence of local availability.

Future heightfield import must preserve horizontal and vertical datum metadata and distinguish ellipsoidal height from orthometric elevation. Future floating-origin work must transform all coupled state consistently. Save/resume tests, origin-shift invariance tests and real terrain seam tests are separate acceptance cases; passing coordinate algebra tests does not establish these integrations.

Large engine/SDK installation, account creation and paid services require the user's explicit approval under the project request. Those dependencies do not block writing and testing the standalone core.
