#include "aether/session.hpp"
#include <charconv>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <locale>
#include <map>
#include <sstream>
#include <system_error>
#ifdef _WIN32
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

namespace aether {
namespace {
using Fields=std::map<std::string,std::string>;
std::string trim(const std::string& text) {
    const auto first=text.find_first_not_of(" \t\r\n");
    if(first==std::string::npos) return {};
    return text.substr(first,text.find_last_not_of(" \t\r\n")-first+1);
}
Fields readFields(const std::filesystem::path& path) {
    if(std::filesystem::file_size(path)>32768) throw std::invalid_argument("Input exceeds 32 KiB limit");
    std::ifstream file(path);
    if(!file) throw std::runtime_error("Cannot open input file");
    Fields values; std::string line; std::size_t number=0;
    while(std::getline(file,line)) {
        ++number; line=trim(line);
        if(line.empty() || line.front()=='#') continue;
        const auto equal=line.find('=');
        if(equal==std::string::npos) throw std::invalid_argument("Missing '=' at line "+std::to_string(number));
        auto key=trim(line.substr(0,equal)),value=trim(line.substr(equal+1));
        if(key.empty()||value.empty()||!values.emplace(key,value).second)
            throw std::invalid_argument("Empty or duplicate field at line "+std::to_string(number));
    }
    if(file.bad()) throw std::runtime_error("Input read failure");
    return values;
}
std::string take(Fields& fields,const std::string& key) {
    const auto found=fields.find(key);
    if(found==fields.end()) throw std::invalid_argument("Missing field: "+key);
    auto result=found->second; fields.erase(found); return result;
}
double number(Fields& fields,const std::string& key) {
    const auto text=take(fields,key); double value=0;
    const auto parsed=std::from_chars(text.data(),text.data()+text.size(),value);
    if(parsed.ec!=std::errc{} || parsed.ptr!=text.data()+text.size() || !std::isfinite(value))
        throw std::invalid_argument("Invalid finite number: "+key);
    return value;
}
std::uint64_t integer(Fields& fields,const std::string& key) {
    const auto text=take(fields,key); std::uint64_t value=0;
    const auto parsed=std::from_chars(text.data(),text.data()+text.size(),value);
    if(parsed.ec!=std::errc{} || parsed.ptr!=text.data()+text.size()) throw std::invalid_argument("Invalid integer: "+key);
    return value;
}
void exact(Fields& fields,const std::string& key,const std::string& expected) {
    if(take(fields,key)!=expected) throw std::invalid_argument("Unsupported value for "+key);
}
void exhausted(const Fields& fields) { if(!fields.empty()) throw std::invalid_argument("Unknown field: "+fields.begin()->first); }
void validId(const std::string& id) {
    if(id.empty() || id.size()>48 || id.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_")!=std::string::npos)
        throw std::invalid_argument("Invalid aircraft identifier");
}
using ConfigField=std::pair<const char*,double AircraftConfig::*>;
constexpr ConfigField configFields[]={
 {"empty_mass_kg",&AircraftConfig::empty_mass_kg},{"fuel_capacity_kg",&AircraftConfig::fuel_capacity_kg},
 {"wing_area_m2",&AircraftConfig::wing_area_m2},{"wing_span_m",&AircraftConfig::wing_span_m},
 {"mean_chord_m",&AircraftConfig::mean_chord_m},{"cl_zero",&AircraftConfig::cl_zero},
 {"cl_alpha_per_rad",&AircraftConfig::cl_alpha_per_rad},{"cd_zero",&AircraftConfig::cd_zero},
 {"induced_drag_factor",&AircraftConfig::induced_drag_factor},{"stall_angle_rad",&AircraftConfig::stall_angle_rad},
 {"max_power_w",&AircraftConfig::max_power_w},{"prop_efficiency",&AircraftConfig::prop_efficiency},
 {"static_thrust_n",&AircraftConfig::static_thrust_n},{"max_fuel_flow_kg_s",&AircraftConfig::max_fuel_flow_kg_s},
 {"engine_time_constant_s",&AircraftConfig::engine_time_constant_s},{"surface_rate_per_s",&AircraftConfig::surface_rate_per_s},
 {"gear_height_m",&AircraftConfig::gear_height_m},{"gear_spring_n_m",&AircraftConfig::gear_spring_n_m},
 {"gear_damper_ns_m",&AircraftConfig::gear_damper_ns_m},{"rolling_friction",&AircraftConfig::rolling_friction},
 {"brake_friction",&AircraftConfig::brake_friction}};
Vec3 readVector(Fields& f,const std::string& prefix) { return {number(f,prefix+".x"),number(f,prefix+".y"),number(f,prefix+".z")}; }
void writeVector(std::ostream& out,const std::string& prefix,Vec3 v) { out<<prefix<<".x="<<v.x<<'\n'<<prefix<<".y="<<v.y<<'\n'<<prefix<<".z="<<v.z<<'\n'; }
AircraftDefinition readAircraft(Fields& f,const std::string& prefix) {
    AircraftDefinition aircraft; aircraft.id=take(f,prefix+"id"); validId(aircraft.id);
    exact(f,prefix+"provenance","SIMULATION_ESTIMATE");
    for(const auto& [name,member]:configFields) aircraft.config.*member=number(f,prefix+name);
    aircraft.config.inertia_kg_m2=readVector(f,prefix+"inertia_kg_m2");
    validateAircraftConfig(aircraft.config); return aircraft;
}
void writeAircraft(std::ostream& out,const AircraftDefinition& aircraft,const std::string& prefix) {
    out<<prefix<<"id="<<aircraft.id<<'\n'<<prefix<<"provenance=SIMULATION_ESTIMATE\n";
    for(const auto& [name,member]:configFields) out<<prefix<<name<<'='<<aircraft.config.*member<<'\n';
    writeVector(out,prefix+"inertia_kg_m2",aircraft.config.inertia_kg_m2);
}
void writeSession(std::ostream& out,const FlightSession& s) {
    out.imbue(std::locale::classic()); out<<std::setprecision(17);
    out<<"format=AETHER_SESSION\nversion=1\nticks="<<s.ticks<<'\n';
    writeAircraft(out,s.aircraft,"aircraft.");
    out<<"origin.lat="<<s.origin.latitude_deg<<"\norigin.lon="<<s.origin.longitude_deg<<"\norigin.ellipsoid_height_m="<<s.origin.ellipsoid_height_m<<'\n';
    writeVector(out,"position_ned_m",s.state.rigid.position_ned_m);
    writeVector(out,"velocity_ned_mps",s.state.rigid.velocity_ned_mps);
    writeVector(out,"omega_body_rad_s",s.state.rigid.angular_velocity_body_rad_s);
    const auto q=s.state.rigid.orientation_body_to_ned;
    out<<"q.w="<<q.w<<"\nq.x="<<q.x<<"\nq.y="<<q.y<<"\nq.z="<<q.z<<'\n';
    out<<"fuel_kg="<<s.state.fuel_kg<<"\nengine_spool="<<s.state.engine_spool<<'\n';
    out<<"surface.elevator="<<s.state.surfaces.elevator<<"\nsurface.aileron="<<s.state.surfaces.aileron<<"\nsurface.rudder="<<s.state.surfaces.rudder<<"\nsurface.flaps="<<s.state.surfaces.flaps<<'\n';
    out<<"control.elevator="<<s.controls.elevator<<"\ncontrol.aileron="<<s.controls.aileron<<"\ncontrol.rudder="<<s.controls.rudder<<"\ncontrol.throttle="<<s.controls.throttle<<"\ncontrol.flaps="<<s.controls.flaps<<"\ncontrol.brakes="<<s.controls.brakes<<'\n';
    writeVector(out,"wind_ned_mps",s.environment.wind_ned_mps);
    out<<"reference_altitude_m="<<s.environment.reference_altitude_m<<"\nground_enabled="<<(s.environment.ground_enabled?1:0)<<"\nground_down_m="<<s.environment.ground_down_m<<'\n';
}
}
void validateSession(const FlightSession& s) {
    validId(s.aircraft.id); validateAircraftConfig(s.aircraft.config); validateFlightState(s.state,s.aircraft.config);
    validateFlightControls(s.controls); validateFlightEnvironment(s.environment);
    (void)CoordinateTransform::toEcef(s.origin);
    if(s.ticks>120ULL*86400ULL*365ULL) throw std::invalid_argument("Session time limit exceeded");
    if(norm(s.state.rigid.position_ned_m)>50000.0) throw std::invalid_argument("Local prototype radius exceeded (50 km); global flight dynamics not implemented");
    (void)flightInstruments(s.state,s.aircraft.config,s.controls,s.environment);
}
void FlightSession::advance() {
    validateSession(*this);
    auto candidate=*this;
    FlightIntegrator::step(candidate.state,aircraft.config,controls,environment,fixed_dt_s);
    ++candidate.ticks; validateSession(candidate); *this=candidate;
}
GeodeticCoordinates FlightSession::position() const {
    const auto p=state.rigid.position_ned_m;
    return CoordinateTransform::fromEcef(LocalFrame(origin).toEcef({p.y,p.x,-p.z}));
}
AircraftDefinition loadAircraft(const std::filesystem::path& path) {
    auto f=readFields(path); exact(f,"format","AETHER_AIRCRAFT"); exact(f,"version","1");
    auto result=readAircraft(f,""); exhausted(f); return result;
}
FlightSession loadSession(const std::filesystem::path& path) {
    auto f=readFields(path); exact(f,"format","AETHER_SESSION"); exact(f,"version","1");
    FlightSession s; s.ticks=integer(f,"ticks"); s.aircraft=readAircraft(f,"aircraft.");
    s.origin={number(f,"origin.lat"),number(f,"origin.lon"),number(f,"origin.ellipsoid_height_m")};
    s.state.rigid.position_ned_m=readVector(f,"position_ned_m");
    s.state.rigid.velocity_ned_mps=readVector(f,"velocity_ned_mps");
    s.state.rigid.angular_velocity_body_rad_s=readVector(f,"omega_body_rad_s");
    s.state.rigid.orientation_body_to_ned={number(f,"q.w"),number(f,"q.x"),number(f,"q.y"),number(f,"q.z")};
    s.state.fuel_kg=number(f,"fuel_kg"); s.state.engine_spool=number(f,"engine_spool");
    s.state.surfaces={number(f,"surface.elevator"),number(f,"surface.aileron"),number(f,"surface.rudder"),number(f,"surface.flaps")};
    s.controls={number(f,"control.elevator"),number(f,"control.aileron"),number(f,"control.rudder"),number(f,"control.throttle"),number(f,"control.flaps"),number(f,"control.brakes")};
    s.environment.wind_ned_mps=readVector(f,"wind_ned_mps"); s.environment.reference_altitude_m=number(f,"reference_altitude_m");
    const auto ground=integer(f,"ground_enabled"); if(ground>1) throw std::invalid_argument("Invalid boolean ground_enabled");
    s.environment.ground_enabled=ground==1; s.environment.ground_down_m=number(f,"ground_down_m");
    exhausted(f); validateSession(s); return s;
}
void saveSession(const FlightSession& s,const std::filesystem::path& path) {
    validateSession(s);
    if(path.empty()) throw std::invalid_argument("Empty save path");
    auto temp=path; temp += ".tmp-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    if(std::filesystem::exists(temp)) throw std::runtime_error("Temporary save path already exists");
    try {
        std::ofstream out(temp,std::ios::binary|std::ios::trunc); if(!out) throw std::runtime_error("Cannot create temporary save");
        writeSession(out,s); out.flush(); if(!out) throw std::runtime_error("Save write failed"); out.close();
        if(out.fail()) throw std::runtime_error("Save close failed");
#ifdef _WIN32
        if(!MoveFileExW(temp.c_str(),path.c_str(),MOVEFILE_REPLACE_EXISTING|MOVEFILE_WRITE_THROUGH))
            throw std::system_error(static_cast<int>(GetLastError()),std::system_category(),"Save replace failed");
#else
        std::filesystem::rename(temp,path);
#endif
    } catch(...) { std::error_code error; std::filesystem::remove(temp,error); throw; }
}
}
