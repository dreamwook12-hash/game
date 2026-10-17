#include "aether/session.hpp"
#include "aether/units.hpp"
#include <algorithm>
#include <charconv>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <sstream>
#include <vector>

namespace {
double parseNumber(const std::string& text) {
    double n=0; const auto r=std::from_chars(text.data(),text.data()+text.size(),n);
    if(r.ec!=std::errc{}||r.ptr!=text.data()+text.size()||!std::isfinite(n)) throw std::invalid_argument("Invalid number: "+text);
    return n;
}
aether::Vec3 triple(const std::string& text) {
    const auto a=text.find(','),b=text.find(',',a==std::string::npos?text.size():a+1);
    if(a==std::string::npos||b==std::string::npos||text.find(',',b+1)!=std::string::npos) throw std::invalid_argument("Expected three comma-separated numbers");
    return {parseNumber(text.substr(0,a)),parseNumber(text.substr(a+1,b-a-1)),parseNumber(text.substr(b+1))};
}
void usage() {
    std::cout<<"PROJECT AETHER 0.1 | C++20 physics prototype | SIMULATION_ESTIMATE\n"
      "Usage: aether_cli [--aircraft data/aircraft/ga01.aether] [--scenario flight|controls|taxi|drop]\n"
      "  --duration SECONDS   Simulated duration (0..3600], whole 1/120 s steps (default 10)\n"
      "  --throttle 0..1 --elevator -1..1 --aileron -1..1 --rudder -1..1\n"
      "  --flaps 0..1 --brakes 0..1 --wind NORTH,EAST,DOWN (m/s)\n"
      "  --origin LAT,LON,ELLIPSOID_HEIGHT_M (default 0,0,0)\n"
      "  --csv PATH           Write 10 Hz state telemetry to a new file\n"
      "  --save PATH          Save versioned complete state; atomic replace\n"
      "  --load PATH          Resume complete state, may override controls/wind\n"
      "  --benchmark          Report measured CPU simulation-step timing, NOT FPS\n"
      "  --help\n"
      "No graphics, real terrain, certified aircraft, autopilot or mobile build.\n"
      "Ground is a synthetic plane. Flight dynamics limited to local radius 50 km.\n"
      "Altitude uses an independent local atmosphere reference; geodetic height is ellipsoidal.\n";
}
void printState(const aether::FlightSession& s) {
    const auto i=aether::flightInstruments(s.state,s.aircraft.config,s.controls,s.environment); const auto geo=s.position();
    std::cout<<std::fixed<<std::setprecision(6)
      <<"aircraft="<<s.aircraft.id<<" ticks="<<s.ticks<<" sim_seconds="<<s.elapsedSeconds()<<'\n'
      <<"atmosphere_altitude_m="<<i.altitude_m<<" TAS_mps="<<i.true_airspeed_mps<<" EAS_mps="<<i.equivalent_airspeed_mps<<" ground_speed_mps="<<i.ground_speed_mps<<'\n'
      <<"roll_deg="<<aether::units::radToDeg(i.roll_rad)<<" pitch_deg="<<aether::units::radToDeg(i.pitch_rad)<<" true_heading_deg="<<aether::units::radToDeg(i.heading_rad)<<'\n'
      <<"latitude_deg="<<geo.latitude_deg<<" longitude_deg="<<geo.longitude_deg<<" ellipsoid_height_m="<<geo.ellipsoid_height_m<<'\n'
      <<"fuel_kg="<<i.fuel_kg<<" engine_power_w="<<i.engine_power_w<<" on_synthetic_ground="<<i.on_ground<<'\n';
}
void record(std::ostream& out,const aether::FlightSession& s) {
    const auto i=aether::flightInstruments(s.state,s.aircraft.config,s.controls,s.environment); const auto g=s.position();
    const auto& r=s.state.rigid;
    out<<std::setprecision(17)<<s.elapsedSeconds()<<','<<r.position_ned_m.x<<','<<r.position_ned_m.y<<','<<r.position_ned_m.z<<','
       <<i.true_airspeed_mps<<','<<i.equivalent_airspeed_mps<<','<<i.altitude_m<<','<<i.vertical_speed_mps<<','
       <<aether::units::radToDeg(i.roll_rad)<<','<<aether::units::radToDeg(i.pitch_rad)<<','<<aether::units::radToDeg(i.heading_rad)<<','
       <<i.engine_power_w<<','<<i.fuel_kg<<','<<i.on_ground<<','<<g.latitude_deg<<','<<g.longitude_deg<<','<<g.ellipsoid_height_m<<'\n';
}
}
int main(int argc,char** argv) {
    try {
        std::map<std::string,std::string> options; bool benchmark=false;
        for(int a=1;a<argc;++a) {
            const std::string key=argv[a]; if(key=="--help") { usage(); return 0; }
            if(key=="--benchmark") { if(benchmark) throw std::invalid_argument("Duplicate --benchmark"); benchmark=true; continue; }
            const std::vector<std::string> valid={"--aircraft","--scenario","--duration","--throttle","--elevator","--aileron","--rudder","--flaps","--brakes","--wind","--origin","--csv","--save","--load"};
            if(std::find(valid.begin(),valid.end(),key)==valid.end()) throw std::invalid_argument("Unknown option: "+key);
            if(a+1>=argc || !options.emplace(key,argv[++a]).second) throw std::invalid_argument("Missing or duplicate option: "+key);
        }
        aether::FlightSession s; std::string scenario="flight";
        if(options.contains("--load")) {
            if(options.contains("--aircraft")||options.contains("--scenario")||options.contains("--origin"))
                throw std::invalid_argument("--load cannot be combined with initial aircraft/scenario/origin");
            s=aether::loadSession(options.at("--load"));
        } else {
            s.aircraft=aether::loadAircraft(options.contains("--aircraft")?options.at("--aircraft"):"data/aircraft/ga01.aether");
            s.state.fuel_kg=s.aircraft.config.fuel_capacity_kg*0.8; s.controls.throttle=0.65;
            if(options.contains("--scenario")) scenario=options.at("--scenario");
            if(scenario=="taxi") {
                s.state.rigid.position_ned_m={0,0,-s.aircraft.config.gear_height_m}; s.state.rigid.velocity_ned_mps={}; s.controls.throttle=0.25;
            } else if(scenario=="drop") {
                s.state.rigid.position_ned_m={0,0,-s.aircraft.config.gear_height_m-0.2}; s.state.rigid.velocity_ned_mps={}; s.controls.throttle=0;
            } else if(scenario!="flight"&&scenario!="controls") throw std::invalid_argument("Unknown scenario");
            if(options.contains("--origin")) { const auto p=triple(options.at("--origin")); s.origin={p.x,p.y,p.z}; }
        }
        const std::pair<const char*,double aether::FlightControls::*> controlOptions[]={
            {"--throttle",&aether::FlightControls::throttle},{"--elevator",&aether::FlightControls::elevator},
            {"--aileron",&aether::FlightControls::aileron},{"--rudder",&aether::FlightControls::rudder},
            {"--flaps",&aether::FlightControls::flaps},{"--brakes",&aether::FlightControls::brakes}};
        for(const auto& [key,member]:controlOptions) if(options.contains(key)) s.controls.*member=parseNumber(options.at(key));
        if(options.contains("--wind")) s.environment.wind_ned_mps=triple(options.at("--wind"));
        const double duration=options.contains("--duration")?parseNumber(options.at("--duration")):10.0;
        if(duration<=0||duration>3600) throw std::invalid_argument("Duration must be in (0,3600] seconds");
        const double rawTicks=duration/aether::FlightSession::fixed_dt_s;
        const auto count=static_cast<std::uint64_t>(std::llround(rawTicks));
        if(count==0||std::abs(rawTicks-static_cast<double>(count))>1e-7) throw std::invalid_argument("Duration must be a whole number of 1/120 s steps");
        aether::validateSession(s);
        std::ofstream csv;
        if(options.contains("--csv")) {
            const auto path=std::filesystem::path(options.at("--csv"));
            if(std::filesystem::exists(path)) throw std::invalid_argument("CSV output already exists; select a new path");
            if(options.contains("--save")&&std::filesystem::absolute(path).lexically_normal()==std::filesystem::absolute(options.at("--save")).lexically_normal())
                throw std::invalid_argument("CSV and save paths must differ");
            csv.open(path); if(!csv) throw std::runtime_error("Cannot create CSV");
            csv<<"sim_seconds,north_m,east_m,down_m,TAS_mps,EAS_mps,atmosphere_altitude_m,vertical_speed_mps,roll_deg,pitch_deg,true_heading_deg,engine_power_w,fuel_kg,on_synthetic_ground,latitude_deg,longitude_deg,ellipsoid_height_m\n";
            record(csv,s);
        }
        std::vector<double> timing; if(benchmark) timing.reserve(static_cast<std::size_t>(count));
        const auto original=s.controls;
        for(std::uint64_t tick=0;tick<count;++tick) {
            if(scenario=="controls") {
                const auto t=s.elapsedSeconds(); s.controls=original;
                s.controls.elevator=std::clamp(original.elevator+(t>=1&&t<2?0.08:0.0),-1.0,1.0);
                s.controls.aileron=std::clamp(original.aileron+(t>=3&&t<4?0.08:0.0),-1.0,1.0);
            }
            const auto start=std::chrono::steady_clock::now(); s.advance();
            if(benchmark) timing.push_back(std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count());
            if(csv.is_open()&&((tick+1)%12==0||tick+1==count)) record(csv,s);
        }
        if(csv.is_open()) { csv.flush(); if(!csv) throw std::runtime_error("CSV write failed"); }
        if(options.contains("--save")) aether::saveSession(s,options.at("--save"));
        printState(s);
        if(benchmark) {
            const double average=std::accumulate(timing.begin(),timing.end(),0.0)/static_cast<double>(timing.size());
            std::sort(timing.begin(),timing.end());
            const auto percentile=[&](double p) { return timing[static_cast<std::size_t>(std::ceil(p*static_cast<double>(timing.size())))-1]; };
            std::cout<<"CPU_SIMULATION_STEP_TIMING samples="<<timing.size()<<" mean_us="<<average<<" p50_us="<<percentile(.5)<<" p95_us="<<percentile(.95)<<" p99_us="<<percentile(.99)<<"\nRendering_FPS=NOT_MEASURED GPU_time=NOT_MEASURED\n";
        }
        return 0;
    } catch(const std::exception& error) { std::cerr<<"AETHER ERROR: "<<error.what()<<'\n'; return 2; }
}
