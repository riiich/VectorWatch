#pragma once

#include "vectorwatch/model/Aircraft.hpp"

#include <string>

namespace vectorwatch {

struct Scenario {
    std::string name;
    std::string description;
    Aircraft aircraftA;
    Aircraft aircraftB;
    bool expectedConflict;

    Scenario(
        const std::string& scenarioName,
        const std::string& scenarioDescription,
        Aircraft firstAircraft,
        Aircraft secondAircraft,
        bool conflictExpected)
        : name(scenarioName),
          description(scenarioDescription),
          aircraftA(firstAircraft),
          aircraftB(secondAircraft),
          expectedConflict(conflictExpected) {}
};

} // namespace vectorwatch
