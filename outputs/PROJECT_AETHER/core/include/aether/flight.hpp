#pragma once
#include "aether/math.hpp"
#include "aether/atmosphere.hpp"

namespace aether {

// All coefficients describe hypothetical aircraft; provenance: SIMULATION_ESTIMATE.
struct AircraftConfig {
    double empty_mass_kg = 1000.0;
    double fuel_capacity_kg = 100.0;
    Vec3 inertia_kg_m2{1300.0, 1800.0, 2500.0};
    double wing_area_m2 = 16.2;
    double wing_span_m = 10.9;
    double mean_chord_m = 1.49;
    double cl_zero = 0.25;
    double cl_alpha_per_rad = 5.0;
    double cd_zero = 0.028;
    double induced_drag_factor = 0.055;
    double stall_angle_rad = 0.28;
    double max_power_w = 135000.0;
    double prop_efficiency = 0.78;
    double static_thrust_n = 2300.0;
    double max_fuel_flow_kg_s = 0.011;
    double engine_time_constant_s = 1.3;
    double surface_rate_per_s = 1.8;
    double gear_height_m = 1.0;
    double gear_spring_n_m = 90000.0;
    double gear_damper_ns_m = 7000.0;
    double rolling_friction = 0.025;
    double brake_friction = 0.65;
};

struct RigidBodyState {
    Vec3 position_ned_m{0.0, 0.0, -1000.0};
    Vec3 velocity_ned_mps{50.0, 0.0, 0.0};
    Quaternion orientation_body_to_ned{};
    Vec3 angular_velocity_body_rad_s{};
};

// Positive elevator/aileron/rudder command nose-up/right-wing-down/nose-right.
struct FlightControls {
    double elevator = 0.0, aileron = 0.0, rudder = 0.0;
    double throttle = 0.0, flaps = 0.0, brakes = 0.0;
};
struct ControlSurfaces {
    double elevator = 0.0, aileron = 0.0, rudder = 0.0, flaps = 0.0;
};
struct FlightState {
    RigidBodyState rigid{};
    double fuel_kg = 80.0;
    double engine_spool = 0.0;
    ControlSurfaces surfaces{};
};
struct FlightEnvironment {
    Vec3 wind_ned_mps{};
    double reference_altitude_m = 0.0;
    bool ground_enabled = true;
    double ground_down_m = 0.0;
};
struct ForceMoment {
    Vec3 force_body_n{};
    Vec3 moment_body_nm{};
};
struct FlightLoads {
    ForceMoment aerodynamic{}, propulsion{}, contact{};
    Vec3 gravity_ned_n{};
    double airspeed_mps = 0.0, alpha_rad = 0.0, beta_rad = 0.0;
    double lift_coefficient = 0.0, drag_coefficient = 0.0;
    double engine_power_w = 0.0, thrust_n = 0.0;
    bool on_ground = false;
};
struct FlightInstruments {
    double altitude_m = 0.0, true_airspeed_mps = 0.0, equivalent_airspeed_mps = 0.0;
    double ground_speed_mps = 0.0, vertical_speed_mps = 0.0;
    double roll_rad = 0.0, pitch_rad = 0.0, heading_rad = 0.0;
    double alpha_rad = 0.0, beta_rad = 0.0, mach = 0.0;
    double engine_power_w = 0.0, fuel_kg = 0.0;
    bool on_ground = false;
};

void validateAircraftConfig(const AircraftConfig& config);
void validateFlightState(const FlightState& state, const AircraftConfig& config);
void validateFlightControls(const FlightControls& controls);
void validateFlightEnvironment(const FlightEnvironment& environment);
// Frozen applied loads, no implicit gravity. NED force and body torque; dt in (0, 0.05].
void integrateRigidBody(RigidBodyState& state, double mass_kg, Vec3 inertia_kg_m2,
                        Vec3 force_ned_n, Vec3 moment_body_nm, double dt_s);
FlightLoads evaluateFlightLoads(const FlightState& state, const AircraftConfig& config,
                                const FlightControls& controls, const FlightEnvironment& environment);
FlightInstruments flightInstruments(const FlightState& state, const AircraftConfig& config,
                                    const FlightControls& controls, const FlightEnvironment& environment);
class FlightIntegrator {
public:
    // Bounded substeps <= 1/240 s; strong exception guarantee (invalid step leaves state unchanged).
    static void step(FlightState& state, const AircraftConfig& config,
                     const FlightControls& controls, const FlightEnvironment& environment, double dt_s);
};

} // namespace aether
