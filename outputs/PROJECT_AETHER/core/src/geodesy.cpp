#include "aether/geodesy.hpp"
#include "aether/units.hpp"
#include <algorithm>

namespace aether {
namespace {
constexpr double a=CoordinateTransform::semi_major_axis_m;
constexpr double f=1.0/CoordinateTransform::inverse_flattening;
constexpr double b=a*(1.0-f);
constexpr double e2=f*(2.0-f);
void validate(GeodeticCoordinates g) {
    if (!std::isfinite(g.latitude_deg)||!std::isfinite(g.longitude_deg)||!std::isfinite(g.ellipsoid_height_m))
        throw std::invalid_argument("Geodetic coordinates must be finite");
    if (g.latitude_deg < -90 || g.latitude_deg > 90 || g.longitude_deg < -180 || g.longitude_deg > 180)
        throw std::out_of_range("Latitude or longitude out of range");
    if (g.ellipsoid_height_m < CoordinateTransform::min_height_m || g.ellipsoid_height_m > CoordinateTransform::max_height_m)
        throw std::out_of_range("Ellipsoid height outside supported [-10000, 1e8] m");
}
}
Vec3 CoordinateTransform::toEcef(GeodeticCoordinates g) {
    validate(g);
    const double lat=units::degToRad(g.latitude_deg),lon=units::degToRad(g.longitude_deg);
    const double s=std::sin(lat),c=std::cos(lat),n=a/std::sqrt(1.0-e2*s*s);
    // Exact pole avoids manufacturing a longitude from roundoff on inverse conversion.
    if (std::abs(g.latitude_deg)==90.0)
        return {0,0,std::copysign(b+g.ellipsoid_height_m,g.latitude_deg)};
    return {(n+g.ellipsoid_height_m)*c*std::cos(lon),
            (n+g.ellipsoid_height_m)*c*std::sin(lon),
            (n*(1.0-e2)+g.ellipsoid_height_m)*s};
}
GeodeticCoordinates CoordinateTransform::fromEcef(Vec3 v) {
    if (!finite(v)) throw std::invalid_argument("ECEF coordinates must be finite");
    const double radius=norm(v);
    if (radius==0) throw std::invalid_argument("Earth center has no geodetic coordinates");
    if (!std::isfinite(radius) || radius>a+max_height_m+1 || radius<b+min_height_m-1)
        throw std::out_of_range("ECEF position outside supported height domain");
    const double p=std::hypot(v.x,v.y);
    double lat=std::atan2(v.z,p*(1.0-e2));
    // Fixed point solves z + e^2 N sin(phi) = p tan(phi). Converges rapidly
    // in our documented shell; singular deep-interior ellipsoid roots are excluded.
    bool converged=false;
    for (int i=0;i<20;++i) {
        const double s=std::sin(lat),n=a/std::sqrt(1.0-e2*s*s);
        const double next=std::atan2(v.z+e2*n*s,p);
        if (std::abs(next-lat)<1e-15) { lat=next; converged=true; break; }
        lat=next;
    }
    if (!converged) throw std::runtime_error("ECEF inverse did not converge");
    const double s=std::sin(lat),c=std::cos(lat);
    // Normal projection avoids p/cos(phi) instability near either pole.
    double height=p*c+v.z*s-a*std::sqrt(1.0-e2*s*s);
    if (height<min_height_m-1e-6 || height>max_height_m+1e-6)
        throw std::out_of_range("ECEF position outside supported height domain");
    height=std::clamp(height,min_height_m,max_height_m);
    return {units::radToDeg(lat),p==0 ? 0.0 : units::radToDeg(std::atan2(v.y,v.x)),height};
}
LocalFrame::LocalFrame(GeodeticCoordinates origin)
    : origin_(origin),ecef_origin_(CoordinateTransform::toEcef(origin)) {
    const double lat=units::degToRad(origin.latitude_deg),lon=units::degToRad(origin.longitude_deg);
    const double sl=std::sin(lon),cl=std::cos(lon),sp=std::sin(lat),cp=std::cos(lat);
    east_={-sl,cl,0}; north_={-cl*sp,-sl*sp,cp}; up_={cl*cp,sl*cp,sp};
}
Vec3 LocalFrame::toEnu(Vec3 ecef) const {
    if (!finite(ecef)) throw std::invalid_argument("ECEF coordinates must be finite");
    const Vec3 delta=ecef-ecef_origin_;
    const Vec3 result{dot(east_,delta),dot(north_,delta),dot(up_,delta)};
    if (!finite(result)) throw std::overflow_error("ENU transform overflow");
    return result;
}
Vec3 LocalFrame::toEcef(Vec3 enu) const {
    if (!finite(enu)) throw std::invalid_argument("ENU coordinates must be finite");
    const Vec3 result=ecef_origin_+east_*enu.x+north_*enu.y+up_*enu.z;
    if (!finite(result)) throw std::overflow_error("ECEF transform overflow");
    return result;
}
} // namespace aether
