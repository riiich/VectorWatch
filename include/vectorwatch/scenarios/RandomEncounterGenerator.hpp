#pragma once

#include "vectorwatch/model/Aircraft.hpp"
#include "vectorwatch/util/Optional.hpp"

#include <cstdint>

namespace vectorwatch {

enum class RandomEncounterOutcome {
    Any,
    Collision,
    Pass,
};

struct RandomEncounter {
    Aircraft aircraftA;
    Aircraft aircraftB;
    Vector3 waypoint;
    Vector3 windVelocity;
    std::uint32_t seed;
    bool expectedCollision{};

    RandomEncounter(
        Aircraft firstAircraft,
        Aircraft secondAircraft,
        Vector3 sharedWaypoint,
        Vector3 wind,
        std::uint32_t encounterSeed,
        bool collisionExpected)
        : aircraftA(firstAircraft),
          aircraftB(secondAircraft),
          waypoint(sharedWaypoint),
          windVelocity(wind),
          seed(encounterSeed),
          expectedCollision(collisionExpected) {}
};

class RandomEncounterGenerator {
public:
    static RandomEncounter generate(
        Optional<std::uint32_t> seed = Optional<std::uint32_t>(),
        RandomEncounterOutcome outcome = RandomEncounterOutcome::Any);
};

} // namespace vectorwatch
