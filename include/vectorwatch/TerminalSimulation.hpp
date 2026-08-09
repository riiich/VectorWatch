#pragma once

#include "vectorwatch/Aircraft.hpp"
#include "vectorwatch/ConflictDetector.hpp"

#include <string_view>

namespace vectorwatch {

struct TerminalSimulationOptions {
    double speedMultiplier{5.0};
    double durationSeconds{60.0};
    double updateRateHz{20.0};
};

void runTerminalSimulation(
    std::string_view scenarioName,
    Aircraft aircraftA,
    Aircraft aircraftB,
    const ConflictDetector& detector,
    TerminalSimulationOptions options = {});

} // namespace vectorwatch
