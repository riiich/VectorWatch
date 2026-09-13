#pragma once

#include "vectorwatch/scenarios/RandomEncounterGenerator.hpp"
#include "vectorwatch/simulation/TerminalSimulationOptions.hpp"
#include "vectorwatch/util/Optional.hpp"

#include <cstdint>
#include <string>

namespace vectorwatch {

struct SimulationCommandOptions {
    std::string scenarioName{"head-on"};
    TerminalSimulationOptions simulation{};
    Optional<std::uint32_t> scenarioSeed{};
    RandomEncounterOutcome requestedOutcome{RandomEncounterOutcome::Any};
};

struct CommandLineParseResult {
    Optional<SimulationCommandOptions> options{};
    std::string error{};
};

CommandLineParseResult parseSimulationCommand(
    int argc,
    char* argv[]);

} // namespace vectorwatch
