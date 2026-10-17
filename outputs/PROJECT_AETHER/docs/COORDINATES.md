# Coordinates, units, atmosphere and simulation time

## Public conventions

The simulation uses SI values internally: meters, seconds, kilograms, newtons, radians and kelvin. Degrees, feet and knots enter only through named conversion functions. A foot is exactly 0.3048 m and a nautical mile is exactly 1852 m. Geodetic latitude/longitude fields are explicitly degrees; ellipsoid height is meters. Operational conversion, orientation, atmosphere and clock functions reject nonfinite or out-of-domain input with exceptions. `Vec3` and quaternion aggregate construction and basic arithmetic are low-level IEEE 754 operations; callers must validate these raw value types before simulation API entry.

| Frame | X | Y | Z | Handedness |
|---|---|---|---|---|
| Aircraft body | Forward | Right | Down | Right |
| Local NED | North | East | Down | Right |
| Local ENU | East | North | Up | Right |
| ECEF | Equator/Greenwich | Equator/90 degrees east | North pole | Right |

`Quaternion` stores `(w,x,y,z)` and uses Hamilton multiplication. It maps body vectors into local NED. Positive roll lowers the right wing, positive pitch raises the nose, and positive yaw turns north toward east. `fromEuler(roll,pitch,yaw)` uses `Rz(yaw) Ry(pitch) Rx(roll)`; arguments are radians. `q.conjugate()` maps NED back to body when `q` is normalized. Rotation normalizes its quaternion, rejects zero/nonfinite quaternions and rejects nonfinite vectors. Internal quaternion integration should normalize every step.

## WGS84 and local origin

The ellipsoid uses semi-major axis 6378137 m and reciprocal flattening 298.257223563. ECEF origin is Earth center, not the runway. The derived semi-minor axis is approximately 6356752.314245 m. Parameters and frame convention follow [NGA WGS84](https://earth-info.nga.mil/index.php?action=wgs84&dir=wgs84).

`CoordinateTransform::toEcef` accepts latitude in [-90,90], longitude in [-180,180] and ellipsoid height in [-10000,100000000] m. The inverse supports the same ellipsoid shell. Earth center and deep-interior ambiguous geodetic roots are explicitly rejected. Exact poles return longitude zero because longitude there is undefined. Either antimeridian endpoint is valid; inverse longitude follows `atan2`, so comparisons must use angular wrapping. Inverse latitude uses a converged fixed-point solution, and height uses normal projection to remain stable at the poles. The implementation throws if convergence fails rather than returning an unverified approximation.

`LocalFrame(origin)` stores an immutable geodetic origin and an orthonormal tangent basis. `toEnu(ecef)` subtracts that origin before projecting onto east/north/up; `toEcef(enu)` performs the inverse. The basis and transpose inverse follow [ESA Navipedia's ECEF/ENU transformation](https://gssc.esa.int/navipedia/index.php/Transformations_between_ECEF_and_ENU_coordinates). At a pole, the origin's chosen longitude defines the local east/north orientation. `nedToEnu` and `enuToNed` both swap X/Y and negate Z. The local frame is a fixed tangent frame; it is not a curved-Earth global flight integrator, terrain projection, geoid model or magnetic compass model. Regional transform accuracy does not establish long-distance dynamics accuracy.

## Standard atmosphere and altitude distinctions

`AtmosphereModel::sample` accepts geopotential altitude in [-500,20000] m. It uses a dry standard atmosphere with a -0.0065 K/m temperature lapse rate through 11000 m and a 216.65 K isothermal layer above that. Hydrostatic pressure is integrated per layer; density follows the ideal gas law and sound speed uses gamma 1.4. Constants are gravity 9.80665 m/s², molecular weight 28.9644 kg/kmol and universal gas constant 8314.32 J/(kmol K). Below sea level, the first layer is extended only to -500 m. These low-altitude equations and reference values follow [U.S. Standard Atmosphere, 1976, NASA-TM-X-74335](https://ntrs.nasa.gov/citations/19770009539).

Ellipsoid height, mean sea level altitude, geopotential altitude, and height above ground are different quantities. This prototype does not contain a geoid model or Earth gravity potential model. A scenario supplies the local origin's atmosphere reference altitude separately from its WGS84 ellipsoid height, and local upward displacement adjusts that reference. This is an explicit regional approximation. No humidity, nonstandard weather profile or forecast data are implied by the standard model.

## Fixed simulation clock

`FixedStepClock` defaults to 1/120 s. `advance(elapsed_seconds)` returns `ClockAdvance { steps, step_seconds, interpolation_alpha, dropped_seconds }`. Run the dynamics exactly `steps` times using `step_seconds`; interpolation is for presentation only. By default, at most eight steps run per call. Excess whole ticks are dropped and reported, while the fractional remainder is retained. This prevents unbounded catch-up after a paused or stalled render loop. A one-second frame therefore executes eight ticks and reports 112/120 seconds dropped. This policy slows the simulated elapsed timeline relative to wall time during a stall; dropped time is never silently integrated with a large timestep.

The clock supports timesteps [1e-6,1] s, catch-up limits [1,1000000], and per-call elapsed time [0,86400] s. It uses an integer executed-tick count, double duration values and extended-precision accumulation where the target ABI provides it. Replaying identical elapsed inputs on the same build is deterministic; bitwise cross-platform identity is not promised. `reset()` clears accumulated time, tick count and dropped-time accounting. Save files must preserve simulation state and tick count separately; this clock is not a wall-clock timestamp source.

## Validation scope

`tests/foundation.cpp` checks independent equator/pole and local-axis anchors, a latitude/longitude/height grid including antimeridian and negative height, quaternion direction/composition/inverse properties, standard atmosphere reference values and layer continuity, unit round trips, invalid input handling, 120 Hz accumulation, catch-up accounting and repeatability. These tests establish implementation consistency with the listed model and conventions. They do not validate aircraft aerodynamics, geoid accuracy, real terrain, GUI frame rates or mobile execution.
