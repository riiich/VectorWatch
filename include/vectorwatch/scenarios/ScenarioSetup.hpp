#pragma once
#include "vectorwatch/scenarios/RandomEncounterGenerator.hpp"
#include "vectorwatch/simulation/SimulationSession.hpp"
#include <string>
namespace vectorwatch {
struct ScenarioSetup {
    Aircraft aircraftA;
    Aircraft aircraftB;
    double durationSeconds{60.0};
    Vector3 windVelocity{};
    Optional<Vector3> waypoint{};
    Optional<WorldBounds> worldBounds{};
    Optional<std::uint32_t> seed{};
};
ScenarioSetup prepareScenario(const std::string& name,
    Optional<std::uint32_t> seed = {},
    RandomEncounterOutcome outcome = RandomEncounterOutcome::Any);
}
