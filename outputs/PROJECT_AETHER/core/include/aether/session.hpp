#pragma once
#include "aether/flight.hpp"
#include "aether/geodesy.hpp"
#include <cstdint>
#include <filesystem>
#include <string>

namespace aether {
struct AircraftDefinition {
    std::string id = "AETHER-GA01";
    AircraftConfig config{};
};
// Origin height is ellipsoidal. atmosphere reference_altitude_m is independently specified.
struct FlightSession {
    AircraftDefinition aircraft{};
    GeodeticCoordinates origin{0.0,0.0,0.0};
    FlightState state{};
    FlightControls controls{};
    FlightEnvironment environment{};
    std::uint64_t ticks = 0;
    static constexpr double fixed_dt_s = 1.0/120.0;
    void advance();
    double elapsedSeconds() const { return static_cast<double>(ticks)*fixed_dt_s; }
    GeodeticCoordinates position() const;
};
AircraftDefinition loadAircraft(const std::filesystem::path& path);
FlightSession loadSession(const std::filesystem::path& path);
void saveSession(const FlightSession& session, const std::filesystem::path& path);
void validateSession(const FlightSession& session);
}
