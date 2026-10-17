# PROJECT AETHER development rules

1. Inspect `git status` before edits. This delivery initially has no Git repository.
2. Preserve user changes and assets; do not discard uncommitted work.
3. Keep physics independent of rendering and device APIs.
4. Use SI internally; conversions belong at boundaries.
5. Document axes, handedness, units and transform directions.
6. Record aerodynamic data provenance; GA01/GA02 are SIMULATION_ESTIMATE.
7. Check external asset and dataset licenses before use.
8. Never commit keys or secrets.
9. Implement small coherent units and verify each.
10. Never report PASS for an unexecuted test.
11. Preserve failed build logs and explain corrections.
12. Run relevant tests after changes.
13. Separate build, execution, GUI and target-device results.
14. Keep docs aligned with actual code.
15. Validate Windows, Android and iOS separately.
16. Optimize from measurements; CPU throughput is not rendering FPS.
17. Ignore caches, temporary state and generated build files in Git.
18. No purchases, deletion or destructive changes without authorization.
19. Record concrete next work and blockers.
20. Do not claim completed product or real-aircraft validation.

Build: `./scripts/build.ps1 -ZigPath <absolute-path-to-zig.exe>`
Tests: `./bin/aether_tests.exe`; CLI: `./bin/aether_cli.exe --help`.
Portable CMake path: `cmake -S . -B build` then `cmake --build build` and `ctest --test-dir build --output-on-failure` (requires separately installed tools).

Public core APIs reject nonfinite or out-of-domain inputs with exceptions. Body is right-handed forward/right/down. Local dynamics use right-handed NED; explicit NED/ENU conversion connects to WGS84. Quaternion maps body vectors to NED. Origin ellipsoid height and local atmosphere reference altitude are distinct quantities.
