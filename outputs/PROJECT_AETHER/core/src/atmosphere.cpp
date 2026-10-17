#include "aether/atmosphere.hpp"
#include "aether/math.hpp"

namespace aether {
AtmosphereSample AtmosphereModel::sample(double altitude) {
    if (!std::isfinite(altitude)) throw std::invalid_argument("Atmosphere altitude must be finite");
    if (altitude<min_altitude_m || altitude>max_altitude_m)
        throw std::out_of_range("Atmosphere altitude outside [-500, 20000] geopotential meters");
    constexpr double gas_constant=8314.32/28.9644;
    constexpr double gamma=1.4,t0=288.15,p0=101325.0,lapse=-0.0065,tropopause=11000.0;
    constexpr double t11=t0+lapse*tropopause;
    double temperature=0,pressure=0;
    if (altitude<=tropopause) {
        temperature=t0+lapse*altitude;
        pressure=p0*std::pow(temperature/t0,-gravity/(gas_constant*lapse));
    } else {
        temperature=t11;
        const double p11=p0*std::pow(t11/t0,-gravity/(gas_constant*lapse));
        pressure=p11*std::exp(-gravity*(altitude-tropopause)/(gas_constant*t11));
    }
    return {temperature,pressure,pressure/(gas_constant*temperature),std::sqrt(gamma*gas_constant*temperature)};
}
} // namespace aether
