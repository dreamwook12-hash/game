#include "aether/flight.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace aether {
namespace {
void range(double value, double low, double high, const char* name) {
    if (!std::isfinite(value) || value < low || value > high)
        throw std::invalid_argument(std::string(name) + " outside supported finite range");
}
void vectorRange(Vec3 value, double maximum, const char* name) {
    range(value.x, -maximum, maximum, name);
    range(value.y, -maximum, maximum, name);
    range(value.z, -maximum, maximum, name);
}
void validateInertia(Vec3 inertia) {
    range(inertia.x, 0.01, 1e10, "inertia x");
    range(inertia.y, 0.01, 1e10, "inertia y");
    range(inertia.z, 0.01, 1e10, "inertia z");
    if (inertia.x > inertia.y + inertia.z || inertia.y > inertia.x + inertia.z ||
        inertia.z > inertia.x + inertia.y)
        throw std::invalid_argument("principal inertia violates triangle inequality");
}
void validateRigid(const RigidBodyState& state) {
    vectorRange(state.position_ned_m, 1e9, "position");
    vectorRange(state.velocity_ned_mps, 10000.0, "velocity");
    vectorRange(state.angular_velocity_body_rad_s, 1000.0, "angular velocity");
    const auto q = state.orientation_body_to_ned;
    if (!finite(q) || std::abs(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z - 1.0) > 1e-6)
        throw std::invalid_argument("orientation must be a finite unit quaternion");
}
void validateDt(double dt) {
    range(dt, 0.0, 0.05, "dt");
    if (dt == 0.0) throw std::invalid_argument("dt must be positive");
}
double approach(double value, double target, double maximum_delta) {
    return value + std::clamp(target - value, -maximum_delta, maximum_delta);
}
Vec3 angularAcceleration(Vec3 omega, Vec3 inertia, Vec3 torque) {
    const Vec3 momentum{inertia.x*omega.x, inertia.y*omega.y, inertia.z*omega.z};
    const Vec3 net = torque - cross(omega, momentum);
    return {net.x/inertia.x, net.y/inertia.y, net.z/inertia.z};
}
void integrateUnchecked(RigidBodyState& state, double mass, Vec3 inertia,
                        Vec3 force, Vec3 torque, double dt) {
    const Vec3 acceleration = force / mass;
    state.position_ned_m += state.velocity_ned_mps*dt + acceleration*(0.5*dt*dt);
    state.velocity_ned_mps += acceleration*dt;
    const Vec3 omega0 = state.angular_velocity_body_rad_s;
    const Vec3 omega_mid = omega0 + angularAcceleration(omega0, inertia, torque)*(0.5*dt);
    state.angular_velocity_body_rad_s += angularAcceleration(omega_mid, inertia, torque)*dt;
    const double rate = norm(omega_mid);
    const double half_angle = 0.5*rate*dt;
    const double scale = rate > 1e-12 ? std::sin(half_angle)/rate : 0.5*dt;
    const Quaternion increment{std::cos(half_angle), omega_mid.x*scale,
                               omega_mid.y*scale, omega_mid.z*scale};
    state.orientation_body_to_ned = (state.orientation_body_to_ned*increment).normalized();
}

FlightLoads evaluateUnchecked(const FlightState& state, const AircraftConfig& config,
                              const FlightControls& controls, const FlightEnvironment& env) {
    FlightLoads loads;
    const auto& rigid = state.rigid;
    const auto q = rigid.orientation_body_to_ned;
    const auto inverse = q.conjugate();
    const auto atmosphere = AtmosphereModel::sample(env.reference_altitude_m-rigid.position_ned_m.z);
    const Vec3 air = inverse.rotate(rigid.velocity_ned_mps-env.wind_ned_mps);
    const double speed = norm(air);
    loads.airspeed_mps = speed;
    loads.alpha_rad = speed > 1e-8 ? std::atan2(air.z, air.x) : 0.0;
    loads.beta_rad = speed > 1e-8 ? std::asin(std::clamp(air.y/speed, -1.0, 1.0)) : 0.0;
    const double alpha = loads.alpha_rad;
    const double beta = loads.beta_rad;
    const double transition = std::clamp((std::abs(alpha)-config.stall_angle_rad)/0.16, 0.0, 1.0);
    const double blend = transition*transition*(3.0-2.0*transition);
    const double attached_cl = config.cl_zero + config.cl_alpha_per_rad*alpha
                               + 0.8*state.surfaces.flaps + 0.25*state.surfaces.elevator;
    const double separated_cl = 0.9*std::sin(2.0*alpha) + 0.15*config.cl_zero*std::cos(alpha);
    loads.lift_coefficient = ((1.0-blend)*attached_cl + blend*separated_cl)*std::cos(beta)*std::cos(beta);
    loads.drag_coefficient = config.cd_zero + config.induced_drag_factor*loads.lift_coefficient*loads.lift_coefficient
                            + 0.08*state.surfaces.flaps + 0.8*std::sin(alpha)*std::sin(alpha)
                            + 0.2*std::sin(beta)*std::sin(beta) + 0.8*blend;
    const double pressure_area = 0.5*atmosphere.density_kg_m3*speed*speed*config.wing_area_m2;
    if (speed > 1e-8) {
        const Vec3 along = air/speed;
        const Vec3 lift_direction{std::sin(alpha), 0.0, -std::cos(alpha)};
        const Vec3 side_direction = cross(along, lift_direction);
        loads.aerodynamic.force_body_n = along*(-pressure_area*loads.drag_coefficient)
                                        + lift_direction*(pressure_area*loads.lift_coefficient)
                                        + side_direction*(-pressure_area*0.65*beta);
        const Vec3 omega = rigid.angular_velocity_body_rad_s;
        const double inverse_speed = 1.0/std::max(speed, 3.0);
        const double p = omega.x*config.wing_span_m*0.5*inverse_speed;
        const double pitch_rate = omega.y*config.mean_chord_m*0.5*inverse_speed;
        const double r = omega.z*config.wing_span_m*0.5*inverse_speed;
        const double roll = 0.16*state.surfaces.aileron-0.10*beta-0.45*p+0.05*r;
        const double pitch = 0.035-0.65*alpha+0.70*state.surfaces.elevator
                             -0.08*state.surfaces.flaps-8.0*pitch_rate;
        const double yaw = 0.10*beta+0.13*state.surfaces.rudder-0.20*r-0.03*p;
        loads.aerodynamic.moment_body_nm = {pressure_area*config.wing_span_m*roll,
                                            pressure_area*config.mean_chord_m*pitch,
                                            pressure_area*config.wing_span_m*yaw};
    }
    if (state.fuel_kg > 0.0 && state.engine_spool > 0.0) {
        const double density_ratio = atmosphere.density_kg_m3/1.225;
        loads.engine_power_w = config.max_power_w*state.engine_spool*std::pow(density_ratio, 0.8);
        const double static_thrust = config.static_thrust_n*state.engine_spool*std::pow(density_ratio, 0.6);
        const double characteristic_speed = config.prop_efficiency*loads.engine_power_w/static_thrust;
        const double advance = std::max(0.0, air.x)/characteristic_speed;
        loads.thrust_n = static_thrust/std::sqrt(1.0+advance*advance);
        loads.propulsion.force_body_n = {loads.thrust_n, 0.0, 0.0};
    }
    loads.gravity_ned_n = {0.0, 0.0, (config.empty_mass_kg+state.fuel_kg)*gravity};
    if (env.ground_enabled) {
        const std::array<Vec3,3> feet{{{1.2*config.mean_chord_m, 0.0, config.gear_height_m},
                                      {-0.35*config.mean_chord_m, -0.12*config.wing_span_m, config.gear_height_m},
                                      {-0.35*config.mean_chord_m, 0.12*config.wing_span_m, config.gear_height_m}}};
        Vec3 forward = q.rotate({1.0, 0.0, 0.0});
        forward.z = 0.0;
        forward = norm(forward) > 1e-8 ? forward/norm(forward) : Vec3{1.0,0.0,0.0};
        const Vec3 right{-forward.y,forward.x,0.0};
        for (const Vec3 foot : feet) {
            const Vec3 position = rigid.position_ned_m+q.rotate(foot);
            const double penetration = position.z-env.ground_down_m;
            if (penetration <= 0.0) continue;
            const Vec3 velocity = rigid.velocity_ned_mps+q.rotate(cross(rigid.angular_velocity_body_rad_s, foot));
            const double normal = std::max(0.0, config.gear_spring_n_m*penetration+config.gear_damper_ns_m*velocity.z);
            loads.on_ground = loads.on_ground || normal > 0.0;
            const double along_slip = std::tanh(dot(velocity, forward)/0.25);
            const double side_slip = std::tanh(dot(velocity, right)/0.25);
            const double slip_scale = std::max(1.0, std::hypot(along_slip, side_slip));
            const double longitudinal_mu = config.rolling_friction+controls.brakes*config.brake_friction;
            const double lateral_mu = std::max(0.6, config.brake_friction);
            const Vec3 contact_ned = Vec3{0.0,0.0,-normal}
                                      - forward*(normal*longitudinal_mu*along_slip/slip_scale)
                                      - right*(normal*lateral_mu*side_slip/slip_scale);
            const Vec3 body_force = inverse.rotate(contact_ned);
            loads.contact.force_body_n += body_force;
            loads.contact.moment_body_nm += cross(foot, body_force);
        }
    }
    return loads;
}
} // namespace

void validateAircraftConfig(const AircraftConfig& c) {
    range(c.empty_mass_kg, 1.0, 1e6, "empty mass");
    range(c.fuel_capacity_kg, 0.0, 1e6, "fuel capacity");
    validateInertia(c.inertia_kg_m2);
    range(c.wing_area_m2, 0.01, 10000.0, "wing area");
    range(c.wing_span_m, 0.01, 1000.0, "wing span");
    range(c.mean_chord_m, 0.01, 1000.0, "mean chord");
    range(c.cl_zero, -2.0, 2.0, "CL zero");
    range(c.cl_alpha_per_rad, 0.0, 20.0, "CL alpha");
    range(c.cd_zero, 0.0, 5.0, "CD zero");
    range(c.induced_drag_factor, 0.0, 5.0, "induced drag");
    range(c.stall_angle_rad, 0.05, 0.6, "stall angle");
    range(c.max_power_w, 1.0, 1e9, "maximum power");
    range(c.prop_efficiency, 0.01, 1.0, "prop efficiency");
    range(c.static_thrust_n, 0.01, 1e8, "static thrust");
    range(c.max_fuel_flow_kg_s, 0.0, 1000.0, "fuel flow");
    range(c.engine_time_constant_s, 0.01, 100.0, "engine time constant");
    range(c.surface_rate_per_s, 0.01, 100.0, "surface rate");
    range(c.gear_height_m, 0.01, 100.0, "gear height");
    range(c.gear_spring_n_m, 0.0, 1e8, "gear spring");
    range(c.gear_damper_ns_m, 0.0, 1e8, "gear damper");
    range(c.rolling_friction, 0.0, 2.0, "rolling friction");
    range(c.brake_friction, 0.0, 2.0, "brake friction");
}
void validateFlightState(const FlightState& state, const AircraftConfig& config) {
    validateAircraftConfig(config);
    validateRigid(state.rigid);
    range(state.fuel_kg, 0.0, config.fuel_capacity_kg, "fuel");
    range(state.engine_spool, 0.0, 1.0, "engine spool");
    range(state.surfaces.elevator, -1.0, 1.0, "elevator surface");
    range(state.surfaces.aileron, -1.0, 1.0, "aileron surface");
    range(state.surfaces.rudder, -1.0, 1.0, "rudder surface");
    range(state.surfaces.flaps, 0.0, 1.0, "flaps surface");
}
void validateFlightControls(const FlightControls& controls) {
    range(controls.elevator, -1.0, 1.0, "elevator command");
    range(controls.aileron, -1.0, 1.0, "aileron command");
    range(controls.rudder, -1.0, 1.0, "rudder command");
    range(controls.throttle, 0.0, 1.0, "throttle command");
    range(controls.flaps, 0.0, 1.0, "flaps command");
    range(controls.brakes, 0.0, 1.0, "brakes command");
}
void validateFlightEnvironment(const FlightEnvironment& env) {
    vectorRange(env.wind_ned_mps, 500.0, "wind");
    range(env.reference_altitude_m, -500.0, 20000.0, "reference atmosphere altitude");
    range(env.ground_down_m, -1e6, 1e6, "ground down");
}
void integrateRigidBody(RigidBodyState& state, double mass, Vec3 inertia,
                        Vec3 force, Vec3 torque, double dt) {
    validateRigid(state);
    range(mass, 0.01, 1e9, "mass");
    validateInertia(inertia);
    vectorRange(force, 1e15, "force");
    vectorRange(torque, 1e15, "torque");
    validateDt(dt);
    auto next = state;
    integrateUnchecked(next, mass, inertia, force, torque, dt);
    validateRigid(next);
    state = next;
}
FlightLoads evaluateFlightLoads(const FlightState& state, const AircraftConfig& config,
                                const FlightControls& controls, const FlightEnvironment& environment) {
    validateFlightState(state, config);
    validateFlightControls(controls);
    validateFlightEnvironment(environment);
    return evaluateUnchecked(state, config, controls, environment);
}
FlightInstruments flightInstruments(const FlightState& state, const AircraftConfig& config,
                                    const FlightControls& controls, const FlightEnvironment& env) {
    const auto loads = evaluateFlightLoads(state, config, controls, env);
    FlightInstruments result;
    result.altitude_m = env.reference_altitude_m-state.rigid.position_ned_m.z;
    const auto atmosphere = AtmosphereModel::sample(result.altitude_m);
    const auto q = state.rigid.orientation_body_to_ned;
    result.roll_rad = std::atan2(2.0*(q.w*q.x+q.y*q.z), 1.0-2.0*(q.x*q.x+q.y*q.y));
    result.pitch_rad = std::asin(std::clamp(2.0*(q.w*q.y-q.z*q.x), -1.0, 1.0));
    result.heading_rad = std::atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z));
    if (result.heading_rad < 0.0) result.heading_rad += 2.0*pi;
    result.true_airspeed_mps = loads.airspeed_mps;
    result.equivalent_airspeed_mps = loads.airspeed_mps*std::sqrt(atmosphere.density_kg_m3/1.225);
    result.ground_speed_mps = std::hypot(state.rigid.velocity_ned_mps.x, state.rigid.velocity_ned_mps.y);
    result.vertical_speed_mps = -state.rigid.velocity_ned_mps.z;
    result.alpha_rad = loads.alpha_rad;
    result.beta_rad = loads.beta_rad;
    result.mach = loads.airspeed_mps/atmosphere.speed_of_sound_mps;
    result.engine_power_w = loads.engine_power_w;
    result.fuel_kg = state.fuel_kg;
    result.on_ground = loads.on_ground;
    return result;
}
void FlightIntegrator::step(FlightState& state, const AircraftConfig& config,
                            const FlightControls& controls, const FlightEnvironment& env, double dt) {
    validateFlightState(state, config);
    validateFlightControls(controls);
    validateFlightEnvironment(env);
    validateDt(dt);
    auto next = state;
    const int count = static_cast<int>(std::ceil(dt*240.0));
    const double h = dt/static_cast<double>(count);
    for (int i=0; i<count; ++i) {
        const double travel = config.surface_rate_per_s*h;
        next.surfaces.elevator = approach(next.surfaces.elevator, controls.elevator, travel);
        next.surfaces.aileron = approach(next.surfaces.aileron, controls.aileron, travel);
        next.surfaces.rudder = approach(next.surfaces.rudder, controls.rudder, travel);
        next.surfaces.flaps = approach(next.surfaces.flaps, controls.flaps, travel*0.25);
        const double target = next.fuel_kg > 0.0 ? controls.throttle : 0.0;
        next.engine_spool = target+(next.engine_spool-target)*std::exp(-h/config.engine_time_constant_s);
        auto loads = evaluateUnchecked(next, config, controls, env);
        const double requested_burn = config.max_fuel_flow_kg_s*(loads.engine_power_w/config.max_power_w)*h;
        const double burn = std::min(next.fuel_kg, requested_burn);
        if (requested_burn > 0.0 && burn < requested_burn) {
            const double fraction = burn/requested_burn;
            loads.propulsion.force_body_n = loads.propulsion.force_body_n*fraction;
            loads.propulsion.moment_body_nm = loads.propulsion.moment_body_nm*fraction;
        }
        const Vec3 body_force = loads.aerodynamic.force_body_n+loads.propulsion.force_body_n+loads.contact.force_body_n;
        const Vec3 torque = loads.aerodynamic.moment_body_nm+loads.propulsion.moment_body_nm+loads.contact.moment_body_nm;
        const double mass = config.empty_mass_kg+next.fuel_kg;
        const Vec3 force = next.rigid.orientation_body_to_ned.rotate(body_force)+loads.gravity_ned_n;
        integrateUnchecked(next.rigid, mass, config.inertia_kg_m2, force, torque, h);
        next.fuel_kg = std::max(0.0, next.fuel_kg-burn);
        validateFlightState(next, config);
        // Reject an out-of-atmosphere step now, rather than storing a state that fails next tick.
        (void)AtmosphereModel::sample(env.reference_altitude_m-next.rigid.position_ned_m.z);
    }
    state = next;
}
} // namespace aether
