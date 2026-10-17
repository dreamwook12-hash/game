#pragma once
#include "aether/math.hpp"

namespace aether::units {
inline double convert(double value, double scale) {
    if (!std::isfinite(value)) throw std::invalid_argument("Unit conversion input must be finite");
    const double converted=value*scale;
    if (!std::isfinite(converted)) throw std::overflow_error("Unit conversion overflow");
    return converted;
}
inline double degToRad(double degrees) { return convert(degrees,pi/180.0); }
inline double radToDeg(double radians) { return convert(radians,180.0/pi); }
inline double feetToMeters(double feet) { return convert(feet,0.3048); }
inline double metersToFeet(double meters) { return convert(meters,1.0/0.3048); }
inline double knotsToMps(double knots) { return convert(knots,1852.0/3600.0); }
inline double mpsToKnots(double mps) { return convert(mps,3600.0/1852.0); }
} // namespace aether::units
