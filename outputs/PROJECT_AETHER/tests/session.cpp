#include "test_support.hpp"
#include "aether/session.hpp"
#include <fstream>
#include <sstream>
#include <limits>

namespace {
std::string read(const std::filesystem::path& p) { std::ifstream in(p); return {std::istreambuf_iterator<char>(in),{}}; }
void write(const std::filesystem::path& p,const std::string& text) { std::ofstream out(p); out<<text; if(!out) throw std::runtime_error("Test file write failed"); }
}
void runSessionTests(TestSuite& suite) {
    using namespace aether;
    const auto scratch=std::filesystem::path("../../work/aether-test-files");
    std::filesystem::create_directories(scratch);
    suite.run("aircraft.strict_load_and_two_distinct_models",[&] {
        const auto a=loadAircraft("data/aircraft/ga01.aether"),b=loadAircraft("data/aircraft/ga02.aether");
        check(a.id!=b.id && b.config.max_power_w>a.config.max_power_w,"distinct definitions");
        FlightState first,second; first.engine_spool=second.engine_spool=1;
        FlightControls controls; controls.throttle=1; FlightEnvironment env;
        for(int i=0;i<120;++i) { FlightIntegrator::step(first,a.config,controls,env,1.0/120); FlightIntegrator::step(second,b.config,controls,env,1.0/120); }
        check(std::abs(norm(first.rigid.velocity_ned_mps)-norm(second.rigid.velocity_ned_mps))>.05,"config changes actual aircraft response");
    });
    suite.run("aircraft.reject_corrupt_missing_duplicate_unknown_version_nonfinite",[&] {
        const auto original=read("data/aircraft/ga01.aether"),path=scratch/"invalid.aether";
        for(const auto& suffix:{std::string("id=duplicate\n"),std::string("unknown=1\n"),std::string("broken line\n")}) {
            write(path,original+suffix); rejects([&]{(void)loadAircraft(path);},"must reject malformed config");
        }
        for(const auto& replacement:{std::string("version=7"),std::string("empty_mass_kg=nan"),std::string("empty_mass_kg=-1"),std::string("empty_mass_kg=1junk")}) {
            auto text=original; const auto key=replacement.substr(0,replacement.find('=')+1); const auto begin=text.find(key),end=text.find('\n',begin);
            text.replace(begin,end-begin,replacement); write(path,text); rejects([&]{(void)loadAircraft(path);},"invalid domain/version/number rejected");
        }
        write(path,"format=AETHER_AIRCRAFT\nversion=1\n"); rejects([&]{(void)loadAircraft(path);},"missing fields");
        write(path,std::string(32769,'x')); rejects([&]{(void)loadAircraft(path);},"oversize");
        rejects([&]{(void)loadAircraft(scratch/"does-not-exist");},"missing file");
    });
    suite.run("session.save_resume_exact_same_build_continuation",[&] {
        FlightSession a; a.controls.throttle=.7; a.controls.elevator=.03; a.environment.wind_ned_mps={3,1,0};
        a.origin={37.0,127.0,123.0};
        for(int i=0;i<300;++i) a.advance();
        const auto path=scratch/"resume.aether"; saveSession(a,path); auto b=loadSession(path);
        for(int i=0;i<300;++i) { a.advance(); b.advance(); }
        saveSession(a,scratch/"a.aether"); saveSession(b,scratch/"b.aether");
        check(read(scratch/"a.aether")==read(scratch/"b.aether"),"serialized state and continuation identical");
        saveSession(a,path); check(loadSession(path).ticks==600,"atomic replacement latest ticks");
    });
    suite.run("session.reject_corruption_preserve_existing_save",[&] {
        FlightSession s; const auto path=scratch/"preserve.aether"; saveSession(s,path); const auto good=read(path);
        s.state.fuel_kg=-1; rejects([&]{saveSession(s,path);},"invalid save rejected before replacement");
        check(read(path)==good,"prior valid file preserved");
        auto corrupt=good; const auto start=corrupt.find("q.w="); corrupt.replace(start,corrupt.find('\n',start)-start,"q.w=0");
        write(scratch/"bad.aether",corrupt); rejects([&]{(void)loadSession(scratch/"bad.aether");},"zero quaternion rejected");
        write(scratch/"bad.aether",good+"ticks=0\n"); rejects([&]{(void)loadSession(scratch/"bad.aether");},"duplicate state rejected");
        write(scratch/"bad.aether",good+"extra=0\n"); rejects([&]{(void)loadSession(scratch/"bad.aether");},"trailing unknown state rejected");
        corrupt=good; const auto v=corrupt.find("version=1"); corrupt.replace(v,9,"version=2");
        write(scratch/"bad.aether",corrupt); rejects([&]{(void)loadSession(scratch/"bad.aether");},"future save rejected");
    });
    suite.run("session.actual_state_to_global_coordinates_and_instruments",[] {
        FlightSession s; s.origin={0,0,150}; s.environment.reference_altitude_m=40;
        s.state.rigid.position_ned_m={0,0,-1000}; const auto g=s.position();
        near(g.ellipsoid_height_m,1150,1e-5,"ellipsoid height independent reference");
        near(flightInstruments(s.state,s.aircraft.config,s.controls,s.environment).altitude_m,1040,1e-9,"atmosphere altitude distinct");
        s.state.rigid.position_ned_m={50001,0,0}; rejects([&]{s.advance();},"local radius limit");
        s.state.rigid.position_ned_m={0,0,-1000}; s.controls.throttle=std::numeric_limits<double>::quiet_NaN();
        rejects([&]{s.advance();},"invalid control propagates failure"); check(s.ticks==0,"failed step does not advance clock");
    });
}
