#include "vectorwatch/scenarios/ScenarioSetup.hpp"
#include "vectorwatch/scenarios/ScenarioCatalog.hpp"
#include <algorithm>
#include <stdexcept>
namespace vectorwatch {
namespace {
WorldBounds encounterWorldBounds(
    const RandomEncounter& encounter) {
    const double minimumX = std::min({
        encounter.aircraftA.position().x,
        encounter.aircraftB.position().x,
        encounter.waypoint.x});
    const double maximumX = std::max({
        encounter.aircraftA.position().x,
        encounter.aircraftB.position().x,
        encounter.waypoint.x});
    const double minimumY = std::min({
        encounter.aircraftA.position().y,
        encounter.aircraftB.position().y,
        encounter.waypoint.y});
    const double maximumY = std::max({
        encounter.aircraftA.position().y,
        encounter.aircraftB.position().y,
        encounter.waypoint.y});
    const double xPadding = std::max((maximumX - minimumX) * 0.1, 1'000.0);
    const double yPadding = std::max((maximumY - minimumY) * 0.1, 1'000.0);
    return WorldBounds(
        minimumX - xPadding,
        maximumX + xPadding,
        minimumY - yPadding,
        maximumY + yPadding);
}

}
ScenarioSetup prepareScenario(const std::string& name, Optional<std::uint32_t> seed,
                              RandomEncounterOutcome outcome) {
    if (name == "random-encounter") {
        const auto encounter = RandomEncounterGenerator::generate(seed, outcome);
        ScenarioSetup setup{encounter.aircraftA, encounter.aircraftB};
        setup.durationSeconds = 120.0;
        setup.windVelocity = encounter.windVelocity;
        setup.waypoint = encounter.waypoint;
        setup.worldBounds = encounterWorldBounds(encounter);
        setup.seed = encounter.seed;
        return setup;
    }
    const auto* scenario = ScenarioCatalog::find(name);
    if (!scenario) throw std::invalid_argument("Unknown scenario");
    return ScenarioSetup{scenario->aircraftA, scenario->aircraftB};
}
}
