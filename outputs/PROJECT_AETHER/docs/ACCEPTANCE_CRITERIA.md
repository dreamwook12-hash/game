# Criteria fixed before implementation

- Unit conversion round trips within 1e-10 relative tolerance for ordinary finite values.
- WGS84 forward anchors: equator radius 6378137 m, polar semi-minor axis 6356752.314245 m (tolerance 1e-5 m).
- Geodetic round trips: angles within 1e-8 degrees (longitude undefined at exact pole); height within 1e-4 m. Cover poles, antimeridian and negative height.
- ECEF/ENU round trips within 1e-7 m at regional scale.
- Invalid coordinates, nonfinite inputs and Earth center rejected explicitly.
- ISA sea level: 288.15 K, 101325 Pa, density 1.225 within 0.001 kg/m^3; 11 km about 216.65 K, 22632 Pa within 2 Pa; explicit domain.
- Fixed clock: 120 steps per second, bounded catch-up, reported dropped time, deterministic same inputs on same build.
- Rigid body: F/m and torque/inertia checked against analytic cases; quaternion unit norm within 1e-10.
- Flight: relative wind changes dynamic pressure, drag dissipates energy, control signs correct, stall lift reduction, bounded static propeller thrust, fuel nonnegative.
- Ground: spring/damper support and friction oppose motion; no ground teleportation; only synthetic flat ground tested.
- Repeatable control-to-force-to-motion-to-instrument path; finite states over bounded scenarios. Numerical consistency does not prove real-aircraft fidelity.
- Config/save: reject unknown versions, missing/duplicate/unknown fields, invalid numbers and trailing data; resume equivalence tested if implemented.
- Rendering, takeoff-to-landing, mobile and real terrain remain NOT RUN until independently exercised and criteria recorded.
