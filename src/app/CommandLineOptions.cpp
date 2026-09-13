#include "vectorwatch/app/CommandLineOptions.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <sstream>
#include <string>
#include <utility>

namespace vectorwatch {
namespace {

bool isFlag(const std::string& text) noexcept {
    return text.size() >= 2 && text[0] == '-' && text[1] == '-';
}

template <typename Integer>
Optional<Integer> parseInteger(const std::string& text) {
    if (text.empty() || text[0] == '-') {
        return Optional<Integer>();
    }
    Integer value{};
    std::istringstream input{text};
    input >> std::noskipws >> value;
    if (!input || !input.eof()) {
        return Optional<Integer>();
    }
    return value;
}

Optional<double> parsePositiveDouble(const std::string& text) {
    double value = 0.0;
    std::istringstream input{text};
    input >> std::noskipws >> value;
    if (!input || !input.eof() || !std::isfinite(value) || value <= 0.0) {
        return Optional<double>();
    }
    return value;
}

CommandLineParseResult error(std::string message) {
    CommandLineParseResult result;
    result.error = std::move(message);
    return result;
}

} // namespace

CommandLineParseResult parseSimulationCommand(int argc, char* argv[]) {
    SimulationCommandOptions command{};
    int index = 2;
    if (index < argc && !isFlag(argv[index])) {
        command.scenarioName = argv[index++];
    }

    // Preserve the original positional [speed] [seed] form.
    if (index < argc && !isFlag(argv[index])) {
        const auto speed = parsePositiveDouble(argv[index++]);
        if (!speed.hasValue()) {
            return error("Simulation speed must be a positive number.");
        }
        command.simulation.speedMultiplier = *speed;
    }
    if (index < argc && !isFlag(argv[index])) {
        const auto seed = parseInteger<std::uint32_t>(argv[index++]);
        if (!seed.hasValue()) {
            return error(
                "Scenario seed must be an integer from 0 to 4294967295.");
        }
        command.scenarioSeed = *seed;
    }

    bool threadsSpecified = false;
    while (index < argc) {
        const std::string flag = argv[index++];
        if (index >= argc) {
            return error(std::string{flag} + " requires a value.");
        }
        const std::string value = argv[index++];

        if (flag == "--speed") {
            const auto speed = parsePositiveDouble(value);
            if (!speed.hasValue()) {
                return error("Simulation speed must be a positive number.");
            }
            command.simulation.speedMultiplier = *speed;
        } else if (flag == "--prediction") {
            if (value == "deterministic") {
                command.simulation.predictionMode = PredictionMode::Deterministic;
            } else if (value == "probabilistic") {
                command.simulation.predictionMode = PredictionMode::Probabilistic;
            } else {
                return error(
                    "Prediction mode must be deterministic or probabilistic.");
            }
        } else if (flag == "--execution") {
            if (value == "sequential") {
                command.simulation.executionMode =
                    SimulationExecutionMode::Sequential;
            } else if (value == "threaded") {
                command.simulation.executionMode =
                    SimulationExecutionMode::Threaded;
            } else {
                return error("Execution mode must be sequential or threaded.");
            }
        } else if (flag == "--threads") {
            threadsSpecified = true;
            if (value == "auto") {
                command.simulation.predictionWorkerCount = 0;
            } else {
                const auto workers = parseInteger<std::size_t>(value);
                if (!workers.hasValue() || *workers == 0) {
                    return error("Thread count must be positive or auto.");
                }
                command.simulation.predictionWorkerCount = *workers;
            }
        } else if (flag == "--samples") {
            const auto samples = parseInteger<std::size_t>(value);
            if (!samples.hasValue() || *samples == 0) {
                return error("Sample count must be positive.");
            }
            command.simulation.predictionSampleCount = *samples;
        } else if (flag == "--scenario-seed") {
            const auto seed = parseInteger<std::uint32_t>(value);
            if (!seed.hasValue()) {
                return error(
                    "Scenario seed must be an integer from 0 to 4294967295.");
            }
            command.scenarioSeed = *seed;
        } else if (flag == "--uncertainty-seed") {
            const auto seed = parseInteger<std::uint32_t>(value);
            if (!seed.hasValue()) {
                return error(
                    "Uncertainty seed must be an integer from 0 to 4294967295.");
            }
            command.simulation.uncertaintySeed = *seed;
        } else if (flag == "--uncertainty") {
            if (value == "low") {
                command.simulation.uncertaintyProfile = UncertaintyProfile::Low;
            } else if (value == "medium") {
                command.simulation.uncertaintyProfile = UncertaintyProfile::Medium;
            } else if (value == "high") {
                command.simulation.uncertaintyProfile = UncertaintyProfile::High;
            } else {
                return error("Uncertainty profile must be low, medium, or high.");
            }
        } else if (flag == "--outcome") {
            if (value == "any") {
                command.requestedOutcome = RandomEncounterOutcome::Any;
            } else if (value == "collision") {
                command.requestedOutcome = RandomEncounterOutcome::Collision;
            } else if (value == "pass") {
                command.requestedOutcome = RandomEncounterOutcome::Pass;
            } else {
                return error("Random outcome must be any, collision, or pass.");
            }
        } else {
            return error("Unknown simulation option: " + std::string{flag});
        }
    }

    if (threadsSpecified &&
        command.simulation.executionMode ==
            SimulationExecutionMode::Sequential) {
        return error("--threads requires --execution threaded.");
    }

    CommandLineParseResult result;
    result.options = command;
    return result;
}

} // namespace vectorwatch
