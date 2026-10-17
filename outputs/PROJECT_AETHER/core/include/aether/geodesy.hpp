#pragma once
#include "aether/math.hpp"

namespace aether {
struct GeodeticCoordinates {
    double latitude_deg=0, longitude_deg=0, ellipsoid_height_m=0;
};
class CoordinateTransform {
public:
    static constexpr double semi_major_axis_m=6378137.0;
    static constexpr double inverse_flattening=298.257223563;
    static constexpr double min_height_m=-10000.0;
    static constexpr double max_height_m=100000000.0;
    static Vec3 toEcef(GeodeticCoordinates geodetic);
    static GeodeticCoordinates fromEcef(Vec3 ecef);
};
class LocalFrame {
public:
    explicit LocalFrame(GeodeticCoordinates origin);
    Vec3 toEnu(Vec3 ecef) const;
    Vec3 toEcef(Vec3 enu) const;
    GeodeticCoordinates origin() const { return origin_; }
private:
    GeodeticCoordinates origin_;
    Vec3 ecef_origin_,east_,north_,up_;
};
inline Vec3 nedToEnu(Vec3 ned) {
    if (!finite(ned)) throw std::invalid_argument("NED vector must be finite");
    return {ned.y,ned.x,-ned.z};
}
inline Vec3 enuToNed(Vec3 enu) {
    if (!finite(enu)) throw std::invalid_argument("ENU vector must be finite");
    return {enu.y,enu.x,-enu.z};
}
} // namespace aether
