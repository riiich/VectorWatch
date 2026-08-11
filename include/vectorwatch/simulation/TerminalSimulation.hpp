#pragma once

#include "vectorwatch/detection/CollisionDetector.hpp"
#include "vectorwatch/detection/ConflictDetector.hpp"
#include "vectorwatch/simulation/TerminalSimulationOptions.hpp"

#include <string_view>

namespace vectorwatch {

class TerminalSimulation {
public:
    explicit TerminalSimulation(const ConflictDetector& detector) noexcept;

    void run(
        std::string_view scenarioName,
        Aircraft aircraftA,
        Aircraft aircraftB,
        TerminalSimulationOptions options = {}) const;

private:
    const ConflictDetector& detector_;
    CollisionDetector collisionDetector_{};
};

} // namespace vectorwatch
