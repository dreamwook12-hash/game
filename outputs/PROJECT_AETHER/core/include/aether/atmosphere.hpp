#pragma once

namespace aether {
struct AtmosphereSample {
    double temperature_k=0,pressure_pa=0,density_kg_m3=0,speed_of_sound_mps=0;
};
class AtmosphereModel {
public:
    static constexpr double min_altitude_m=-500.0,max_altitude_m=20000.0;
    // Dry standard atmosphere; input is geopotential altitude, not ellipsoid height.
    static AtmosphereSample sample(double geopotential_altitude_m);
};
} // namespace aether
