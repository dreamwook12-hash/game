#include "test_support.hpp"
#include "aether/math.hpp"
#include "aether/units.hpp"
#include "aether/geodesy.hpp"
#include "aether/atmosphere.hpp"
#include "aether/clock.hpp"
#include <array>
#include <limits>

using namespace aether;
namespace {
void nearVector(Vec3 actual,Vec3 expected,double tolerance,const std::string& message) {
    near(norm(actual-expected),0,tolerance,message);
}
}
void runFoundationTests(TestSuite& suite) {
    suite.run("units: exact physical anchors and round trips",[] {
        near(units::feetToMeters(1),0.3048,1e-15,"international foot");
        near(units::knotsToMps(3600),1852,1e-10,"nautical mile per hour");
        near(units::degToRad(180),pi,1e-15,"half turn");
        for (double value : {-123456.78,-1.0,0.0,0.01,1.0,76543.21}) {
            const double tolerance=1e-10*std::max(1.0,std::abs(value));
            near(units::radToDeg(units::degToRad(value)),value,tolerance,"angle round trip");
            near(units::metersToFeet(units::feetToMeters(value)),value,tolerance,"length round trip");
            near(units::mpsToKnots(units::knotsToMps(value)),value,tolerance,"speed round trip");
        }
    });
    suite.run("units: reject nonfinite and overflow",[] {
        rejects([]{units::feetToMeters(std::numeric_limits<double>::infinity());},"infinite distance");
        rejects([]{units::degToRad(std::numeric_limits<double>::quiet_NaN());},"NaN angle");
        rejects([]{units::metersToFeet(std::numeric_limits<double>::max());},"distance overflow");
    });
    suite.run("math: right-handed vector and robust norm",[] {
        nearVector(cross({1,0,0},{0,1,0}),{0,0,1},0,"right handed basis");
        near(dot({1,2,3},{4,-5,6}),12,0,"dot product");
        near(norm({3,4,12}),13,1e-14,"vector magnitude");
        near(norm({3e200,4e200,0})/1e200,5,1e-14,"scaled magnitude");
    });
    suite.run("quaternion: FRD to NED signs and composition",[] {
        const auto yaw=Quaternion::fromEuler(0,0,pi/2);
        const auto pitch=Quaternion::fromEuler(0,pi/2,0);
        const auto roll=Quaternion::fromEuler(pi/2,0,0);
        nearVector(yaw.rotate({1,0,0}),{0,1,0},1e-14,"yaw north to east");
        nearVector(pitch.rotate({1,0,0}),{0,0,-1},1e-14,"positive pitch nose up");
        nearVector(roll.rotate({0,1,0}),{0,0,1},1e-14,"positive roll right wing down");
        const Vec3 v{2,-3,4};
        const auto combined=Quaternion::fromEuler(0.2,-0.4,0.7);
        const auto composed=Quaternion::fromEuler(0,0,0.7)*Quaternion::fromEuler(0,-0.4,0)*Quaternion::fromEuler(0.2,0,0);
        nearVector(combined.rotate(v),composed.rotate(v),1e-13,"Euler rotation order");
        nearVector(combined.conjugate().rotate(combined.rotate(v)),v,1e-13,"inverse rotation");
        near(norm(combined.rotate(v)),norm(v),1e-13,"rotation preserves length");
    });
    suite.run("quaternion: robust normalization and invalid input",[] {
        for (double scale : {1e-300,1.0,1e300}) {
            const auto q=Quaternion{scale,scale,-scale,scale}.normalized();
            near(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z,1,1e-15,"unit quaternion");
        }
        rejects([]{Quaternion{0,0,0,0}.normalized();},"zero quaternion");
        rejects([]{Quaternion{0,0,std::numeric_limits<double>::infinity(),0}.normalized();},"infinite quaternion");
        rejects([]{Quaternion::fromEuler(0,std::numeric_limits<double>::quiet_NaN(),0);},"NaN Euler");
        rejects([]{Quaternion{}.rotate({std::numeric_limits<double>::infinity(),0,0});},"infinite vector");
    });
    suite.run("WGS84: equator, poles and antimeridian anchors",[] {
        nearVector(CoordinateTransform::toEcef({0,0,0}),{6378137,0,0},1e-9,"equator Greenwich");
        nearVector(CoordinateTransform::toEcef({0,90,0}),{0,6378137,0},1e-8,"equator east");
        nearVector(CoordinateTransform::toEcef({90,123,0}),{0,0,6356752.314245},1e-5,"north pole");
        nearVector(CoordinateTransform::toEcef({-90,-43,0}),{0,0,-6356752.314245},1e-5,"south pole");
        nearVector(CoordinateTransform::toEcef({0,180,0}),{-6378137,0,0},1e-8,"antimeridian");
        const auto pole=CoordinateTransform::fromEcef({0,0,6356752.314245});
        near(pole.latitude_deg,90,1e-12,"inverse pole latitude");
        near(pole.longitude_deg,0,0,"undefined pole longitude convention");
    });
    suite.run("WGS84: coordinate grid round trips including negative height",[] {
        for (double latitude : {-90.0,-89.999999,-67.2,-1.0,0.0,37.5665,89.999999,90.0})
            for (double longitude : {-180.0,-179.99999,-73.5,0.0,126.978,179.99999,180.0})
                for (double height : {-10000.0,-430.0,0.0,11000.0,100000000.0}) {
                    const GeodeticCoordinates start{latitude,longitude,height};
                    const auto finish=CoordinateTransform::fromEcef(CoordinateTransform::toEcef(start));
                    near(finish.latitude_deg,latitude,1e-8,"latitude round trip");
                    if (std::abs(latitude)!=90.0)
                        near(std::remainder(finish.longitude_deg-longitude,360.0),0,1e-8,"longitude round trip");
                    near(finish.ellipsoid_height_m,height,1e-4,"height round trip");
                }
    });
    suite.run("WGS84: invalid domain and Earth center rejected",[] {
        rejects([]{CoordinateTransform::toEcef({90.001,0,0});},"invalid latitude");
        rejects([]{CoordinateTransform::toEcef({0,180.001,0});},"invalid longitude");
        rejects([]{CoordinateTransform::toEcef({0,0,-10001});},"deep interior height");
        rejects([]{CoordinateTransform::toEcef({0,0,100000001});},"excessive height");
        rejects([]{CoordinateTransform::toEcef({std::numeric_limits<double>::quiet_NaN(),0,0});},"NaN geodetic");
        rejects([]{CoordinateTransform::fromEcef({0,0,0});},"Earth center");
        rejects([]{CoordinateTransform::fromEcef({1,2,3});},"deep interior ECEF");
        rejects([]{CoordinateTransform::fromEcef({1e200,0,0});},"excessive ECEF radius");
        rejects([]{CoordinateTransform::fromEcef({0,std::numeric_limits<double>::infinity(),0});},"infinite ECEF");
    });
    suite.run("local frame: independent equatorial axis anchors",[] {
        const LocalFrame local({0,0,0});
        nearVector(local.toEcef({10,20,30}),{6378167,10,20},1e-10,"ENU axes in ECEF");
        nearVector(local.toEnu({6378167,10,20}),{10,20,30},1e-10,"ECEF axes in ENU");
        nearVector(nedToEnu({2,3,4}),{3,2,-4},0,"NED to ENU axes");
        nearVector(enuToNed(nedToEnu({2,3,4})),{2,3,4},0,"NED round trip");
    });
    suite.run("local frame: regional and polar round trips",[] {
        for (const auto origin : std::array<GeodeticCoordinates,4>{{{37.5665,126.978,35},{-45,179.99,-10},{90,42,0},{-90,-125,800}}}) {
            const LocalFrame local(origin);
            nearVector(local.toEnu(CoordinateTransform::toEcef(origin)),{},1e-9,"local origin");
            for (const auto v : std::array<Vec3,3>{{{1,2,3},{-50000,40000,10000},{100000,-100000,-500}}})
                nearVector(local.toEnu(local.toEcef(v)),v,1e-7,"regional ENU round trip");
        }
        rejects([]{LocalFrame({0,0,0}).toEnu({0,0,std::numeric_limits<double>::quiet_NaN()});},"NaN local input");
    });
    suite.run("atmosphere: reference sea level, tropopause and 20 km",[] {
        const auto sea=AtmosphereModel::sample(0),trop=AtmosphereModel::sample(11000),top=AtmosphereModel::sample(20000);
        near(sea.temperature_k,288.15,1e-10,"sea temperature");
        near(sea.pressure_pa,101325,1e-8,"sea pressure");
        near(sea.density_kg_m3,1.225,0.001,"sea density");
        near(sea.speed_of_sound_mps,340.294,0.01,"sea speed of sound");
        near(trop.temperature_k,216.65,1e-10,"tropopause temperature");
        near(trop.pressure_pa,22632.06,2,"tropopause pressure");
        near(top.pressure_pa,5474.89,1,"20 km pressure");
        near(top.temperature_k,trop.temperature_k,1e-10,"isothermal layer");
    });
    suite.run("atmosphere: continuous layers, monotonic pressure, explicit domain",[] {
        const auto below=AtmosphereModel::sample(10999.999),above=AtmosphereModel::sample(11000.001);
        near(below.pressure_pa,above.pressure_pa,0.01,"pressure continuity");
        near(below.temperature_k,above.temperature_k,1e-5,"temperature continuity");
        double previous=AtmosphereModel::sample(-500).pressure_pa;
        for (int altitude=0;altitude<=20000;altitude+=500) {
            const auto sample=AtmosphereModel::sample(altitude);
            check(sample.pressure_pa<previous,"pressure must fall with altitude");
            check(sample.density_kg_m3>0 && sample.speed_of_sound_mps>0,"positive atmosphere");
            previous=sample.pressure_pa;
        }
        rejects([]{AtmosphereModel::sample(-500.001);},"below atmosphere domain");
        rejects([]{AtmosphereModel::sample(20000.001);},"above atmosphere domain");
        rejects([]{AtmosphereModel::sample(std::numeric_limits<double>::quiet_NaN());},"NaN atmosphere altitude");
    });
    suite.run("clock: 120 Hz across render cadence and fractional elapsed",[] {
        FixedStepClock at60,at1000;
        for (int i=0;i<60;++i) check(at60.advance(1.0/60).steps==2,"two ticks per 60 Hz frame");
        for (int i=0;i<1000;++i) at1000.advance(0.001);
        check(at60.totalSteps()==120 && at1000.totalSteps()==120,"120 ticks in one second");
        near(at60.totalDroppedSeconds(),0,0,"no dropped time at 60 Hz");
        FixedStepClock fractional;
        const auto half=fractional.advance(1.0/240);
        check(half.steps==0,"substep waits"); near(half.interpolation_alpha,0.5,1e-14,"half-step interpolation");
        check(fractional.advance(1.0/240).steps==1,"second half completes tick");
    });
    suite.run("clock: bounded catch-up and exact dropped-time accounting",[] {
        FixedStepClock clock;
        const auto result=clock.advance(1.0);
        check(result.steps==8,"catch-up bounded at eight steps");
        near(result.dropped_seconds,112.0/120,1e-12,"excess time reported");
        near(result.steps*result.step_seconds+result.dropped_seconds,1,1e-12,"time conserved");
        check(clock.advance(0).steps==0,"no excess debt carried over");
        clock.reset(); check(clock.totalSteps()==0,"reset tick counter");
        near(clock.totalDroppedSeconds(),0,0,"reset dropped counter");
    });
    suite.run("clock: deterministic sequence and invalid input",[] {
        FixedStepClock a,b;
        for (double elapsed : {0.001,0.02,0.011,0.25,0.0001,1.0,0.0,0.05}) {
            const auto ra=a.advance(elapsed),rb=b.advance(elapsed);
            check(ra.steps==rb.steps && ra.interpolation_alpha==rb.interpolation_alpha && ra.dropped_seconds==rb.dropped_seconds,"same-input determinism");
            check(ra.interpolation_alpha>=0 && ra.interpolation_alpha<1,"interpolation range");
        }
        rejects([&]{a.advance(-0.1);},"negative elapsed");
        rejects([&]{a.advance(std::numeric_limits<double>::infinity());},"infinite elapsed");
        rejects([&]{a.advance(86401);},"elapsed outside bound");
        rejects([]{FixedStepClock invalid(0);},"zero timestep");
        rejects([]{FixedStepClock invalid(0.01,0);},"zero catch-up bound");
    });
}
