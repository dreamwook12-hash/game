#include "test_support.hpp"
#include "aether/flight.hpp"
#include <limits>

using namespace aether;
namespace {
FlightEnvironment airborne() { FlightEnvironment env; env.ground_enabled=false; return env; }
double quaternionNorm(Quaternion q) { return std::sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z); }
}
void runFlightTests(TestSuite& suite) {
    suite.run("rigid body analytic constant force", [] {
        RigidBodyState state; state.position_ned_m={}; state.velocity_ned_mps={};
        integrateRigidBody(state,10.0,{2,3,4},{20,-40,60},{},0.02);
        near(state.velocity_ned_mps.x,0.04,1e-14,"F/m x");
        near(state.velocity_ned_mps.y,-0.08,1e-14,"F/m y");
        near(state.position_ned_m.z,0.0012,1e-14,"constant acceleration displacement");
    });
    suite.run("rigid body analytic single axis torque", [] {
        RigidBodyState state; state.velocity_ned_mps={};
        integrateRigidBody(state,10.0,{2,3,4},{},{0,12,0},0.02);
        near(state.angular_velocity_body_rad_s.y,0.08,1e-14,"torque/inertia");
        const Vec3 nose=state.orientation_body_to_ned.rotate({1,0,0});
        near(nose.z,-std::sin(0.0008),1e-13,"positive y torque pitches nose up");
    });
    suite.run("rigid body gyroscopic coupling", [] {
        RigidBodyState state; state.angular_velocity_body_rad_s={1,2,3};
        integrateRigidBody(state,10.0,{2,3,4},{},{},1e-5);
        near((state.angular_velocity_body_rad_s.x-1)/1e-5,-3.0,1e-4,"gyro x");
        near((state.angular_velocity_body_rad_s.y-2)/1e-5,2.0,1e-4,"gyro y");
        near((state.angular_velocity_body_rad_s.z-3)/1e-5,-0.5,1e-4,"gyro z");
    });
    suite.run("quaternion constant rotation and normalization", [] {
        RigidBodyState state; state.angular_velocity_body_rad_s={0,0,2};
        for(int i=0;i<1000;++i) integrateRigidBody(state,10.0,{2,2,2},{},{},0.001);
        const Vec3 nose=state.orientation_body_to_ned.rotate({1,0,0});
        near(nose.x,std::cos(2.0),1e-11,"constant angular velocity x");
        near(nose.y,std::sin(2.0),1e-11,"positive yaw turns east");
        near(quaternionNorm(state.orientation_body_to_ned),1.0,1e-12,"unit quaternion");
    });
    suite.run("wind relative dynamic pressure", [] {
        AircraftConfig config; FlightState state; FlightControls controls; auto env=airborne();
        const auto calm=evaluateFlightLoads(state,config,controls,env);
        env.wind_ned_mps={-50,0,0};
        const auto headwind=evaluateFlightLoads(state,config,controls,env);
        near(headwind.airspeed_mps,100.0,1e-12,"headwind relative speed");
        near(headwind.aerodynamic.force_body_n.x,4*calm.aerodynamic.force_body_n.x,1e-8,"quadratic drag");
        near(headwind.aerodynamic.force_body_n.z,4*calm.aerodynamic.force_body_n.z,1e-8,"quadratic lift");
    });
    suite.run("aerodynamic force dissipates relative energy", [] {
        AircraftConfig config; FlightControls controls; auto env=airborne();
        for(const Vec3 velocity : {Vec3{50,0,0},Vec3{45,10,8},Vec3{10,-10,40},Vec3{-50,5,3}}) {
            FlightState state; state.rigid.velocity_ned_mps=velocity;
            const auto loads=evaluateFlightLoads(state,config,controls,env);
            check(dot(loads.aerodynamic.force_body_n,velocity)<0,"aerodynamics do positive work on relative air motion");
        }
    });
    suite.run("control moment signs", [] {
        AircraftConfig config; FlightState state; FlightControls controls; auto env=airborne();
        const auto base=evaluateFlightLoads(state,config,controls,env);
        state.surfaces.elevator=0.5; state.surfaces.aileron=0.5; state.surfaces.rudder=0.5;
        const auto controlled=evaluateFlightLoads(state,config,controls,env);
        check(controlled.aerodynamic.moment_body_nm.x>base.aerodynamic.moment_body_nm.x,"aileron right roll");
        check(controlled.aerodynamic.moment_body_nm.y>base.aerodynamic.moment_body_nm.y,"elevator nose up");
        check(controlled.aerodynamic.moment_body_nm.z>base.aerodynamic.moment_body_nm.z,"rudder nose right");
    });
    suite.run("stall transition reduces lift coefficient", [] {
        AircraftConfig config; FlightState state; FlightControls controls; auto env=airborne();
        state.rigid.velocity_ned_mps={50*std::cos(0.27),0,50*std::sin(0.27)};
        const auto attached=evaluateFlightLoads(state,config,controls,env);
        state.rigid.velocity_ned_mps={50*std::cos(0.55),0,50*std::sin(0.55)};
        const auto stalled=evaluateFlightLoads(state,config,controls,env);
        check(stalled.lift_coefficient<attached.lift_coefficient*0.75,"post stall lift reduction");
        check(stalled.drag_coefficient>attached.drag_coefficient,"post stall drag increase");
    });
    suite.run("propeller finite static thrust and power bound", [] {
        AircraftConfig config; FlightState state; FlightControls controls; auto env=airborne();
        state.rigid.position_ned_m.z=0; state.engine_spool=1.0;
        for(double speed : {0.0,1.0,50.0,150.0}) {
            state.rigid.velocity_ned_mps={speed,0,0};
            const auto loads=evaluateFlightLoads(state,config,controls,env);
            check(std::isfinite(loads.thrust_n)&&loads.thrust_n>0,"finite positive thrust");
            check(loads.thrust_n<=config.static_thrust_n*1.001,"bounded static thrust");
            check(loads.thrust_n*speed<=loads.engine_power_w*config.prop_efficiency+1e-8,"shaft power bound");
        }
    });
    suite.run("surface rate limit and engine spool", [] {
        AircraftConfig config; FlightState state; FlightControls controls; auto env=airborne();
        controls.elevator=1; controls.aileron=-1; controls.flaps=1; controls.throttle=1;
        FlightIntegrator::step(state,config,controls,env,0.05);
        near(state.surfaces.elevator,0.05*config.surface_rate_per_s,1e-14,"elevator rate");
        near(state.surfaces.aileron,-0.05*config.surface_rate_per_s,1e-14,"aileron rate");
        near(state.surfaces.flaps,0.05*config.surface_rate_per_s*0.25,1e-14,"flap rate");
        near(state.engine_spool,1-std::exp(-0.05/config.engine_time_constant_s),1e-14,"spool response");
        check(state.fuel_kg<80.0,"engine burns fuel");
    });
    suite.run("fuel exhaustion disables thrust without negative fuel", [] {
        AircraftConfig config; FlightState state; FlightControls controls; auto env=airborne();
        state.fuel_kg=1e-8; state.engine_spool=1; controls.throttle=1;
        FlightIntegrator::step(state,config,controls,env,0.05);
        near(state.fuel_kg,0.0,0.0,"fuel nonnegative");
        near(evaluateFlightLoads(state,config,controls,env).thrust_n,0.0,0.0,"empty tank thrust");
    });
    suite.run("three point spring damper and wheel braking", [] {
        AircraftConfig config; FlightState state; FlightControls controls; FlightEnvironment env;
        state.rigid.position_ned_m={0,0,-0.95}; state.rigid.velocity_ned_mps={};
        const auto supported=evaluateFlightLoads(state,config,controls,env);
        near(supported.contact.force_body_n.z,-3*config.gear_spring_n_m*0.05,1e-7,"spring support");
        check(supported.on_ground,"contact state");
        state.rigid.velocity_ned_mps={10,0,1};
        const auto moving=evaluateFlightLoads(state,config,controls,env);
        near(moving.contact.force_body_n.z,supported.contact.force_body_n.z-3*config.gear_damper_ns_m,1e-7,"damper support");
        check(moving.contact.force_body_n.x<0,"rolling friction opposes motion");
        controls.brakes=1;
        const auto braked=evaluateFlightLoads(state,config,controls,env);
        check(braked.contact.force_body_n.x<moving.contact.force_body_n.x,"brake increases friction");
        const double previous_height=state.rigid.position_ned_m.z;
        FlightIntegrator::step(state,config,controls,env,1.0/240.0);
        check(std::abs(state.rigid.position_ned_m.z-previous_height)<0.01,"contact must not teleport aircraft");
    });
    suite.run("instruments derive from actual state and wind", [] {
        AircraftConfig config; FlightState state; FlightControls controls; auto env=airborne();
        env.reference_altitude_m=200; env.wind_ned_mps={5,0,0};
        state.rigid.velocity_ned_mps={35,40,-3};
        state.rigid.orientation_body_to_ned=Quaternion::fromEuler(0.2,0.1,-0.5);
        const auto instruments=flightInstruments(state,config,controls,env);
        near(instruments.altitude_m,1200,1e-12,"altitude");
        near(instruments.true_airspeed_mps,std::sqrt(2509.0),1e-10,"wind relative TAS");
        near(instruments.ground_speed_mps,std::hypot(35.0,40.0),1e-10,"ground speed");
        near(instruments.vertical_speed_mps,3,1e-12,"positive climb");
        near(instruments.roll_rad,0.2,1e-12,"roll");
        near(instruments.pitch_rad,0.1,1e-12,"pitch");
        near(instruments.heading_rad,2*pi-0.5,1e-12,"heading wraps positive");
    });
    suite.run("bounded deterministic flight integration", [] {
        AircraftConfig config; FlightState first,second; FlightControls controls; auto env=airborne();
        controls.throttle=0.65; controls.elevator=0.08; env.wind_ned_mps={-4,2,0};
        for(int i=0;i<2400;++i) {
            controls.aileron=i<120 ? 0.05 : 0.0;
            FlightIntegrator::step(first,config,controls,env,1.0/120.0);
            FlightIntegrator::step(second,config,controls,env,1.0/120.0);
        }
        check(finite(first.rigid.position_ned_m)&&finite(first.rigid.velocity_ned_mps),"finite flight state");
        near(norm(first.rigid.position_ned_m-second.rigid.position_ned_m),0,0,"same-build deterministic position");
        near(first.rigid.orientation_body_to_ned.w,second.rigid.orientation_body_to_ned.w,0,"same-build deterministic attitude");
        near(quaternionNorm(first.rigid.orientation_body_to_ned),1,1e-10,"normalized long-run attitude");
        check(norm(first.rigid.velocity_ned_mps)<200,"bounded scenario speed");
    });
    suite.run("flight invalid inputs reject transactionally", [] {
        AircraftConfig config; FlightState state; FlightControls controls; auto env=airborne();
        rejects([&]{FlightIntegrator::step(state,config,controls,env,0);},"zero dt");
        rejects([&]{FlightIntegrator::step(state,config,controls,env,0.1);},"oversized dt");
        controls.throttle=1.1;
        rejects([&]{FlightIntegrator::step(state,config,controls,env,0.01);},"invalid command");
        near(state.rigid.position_ned_m.z,-1000,0,"rejected step preserves state");
        controls.throttle=0; config.inertia_kg_m2={1,1,3};
        rejects([&]{validateAircraftConfig(config);},"impossible inertia");
        config=AircraftConfig{}; state.engine_spool=std::numeric_limits<double>::quiet_NaN();
        rejects([&]{evaluateFlightLoads(state,config,controls,env);},"nonfinite state");
        state=FlightState{}; state.rigid.position_ned_m.z=-19999.99; state.rigid.velocity_ned_mps={0,0,-100};
        rejects([&]{FlightIntegrator::step(state,config,controls,env,0.01);},"atmosphere exit rejected");
        near(state.rigid.position_ned_m.z,-19999.99,0,"failed substep preserves input");
    });
}
