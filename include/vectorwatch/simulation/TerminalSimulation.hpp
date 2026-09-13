#pragma once

#include "vectorwatch/simulation/SimulationSession.hpp"
#include "vectorwatch/simulation/TerminalSimulationOptions.hpp"

namespace vectorwatch {

class TerminalSimulation {
public:
    SimulationResult run(
        Aircraft aircraftA,
        Aircraft aircraftB,
        TerminalSimulationOptions options = {}) const;
};

} // namespace vectorwatch
