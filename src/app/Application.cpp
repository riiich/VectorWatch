#include "vectorwatch/app/Application.hpp"

#include "vectorwatch/app/BenchmarkRunner.hpp"
#include "vectorwatch/app/CommandLineOptions.hpp"
#include "vectorwatch/detection/ConflictDetector.hpp"
#include "vectorwatch/scenarios/ScenarioSetup.hpp"
#include "vectorwatch/scenarios/ScenarioCatalog.hpp"
#include "vectorwatch/simulation/TerminalSimulation.hpp"

#include <iomanip>
#include <iostream>
#include <string>

namespace vectorwatch {
namespace {

void printResult(const Scenario& scenario, const ClosestApproach& result) {
    std::cout << "\nScenario: " << scenario.name << '\n'
              << scenario.description << '\n'
              << std::fixed << std::setprecision(2)
              << "Current 3D separation: "
              << result.currentDistanceMeters / 1'000.0 << " km\n"
              << "TCPA (Time to Closest Point of Approach): "
              << result.timeSeconds << " s\n"
              << "H-CPA (Horizontal Separation at Closest Point of Approach): "
              << result.horizontalSeparationMeters / 1'000.0 << " km\n"
              << "V-CPA (Vertical Separation at Closest Point of Approach): "
              << result.verticalSeparationMeters << " m\n"
              << "Minimum horizontal separation: "
              << result.minimumHorizontalSeparationMeters << " m at "
              << result.minimumHorizontalTimeSeconds << " s\n"
              << "Minimum vertical separation: "
              << result.minimumVerticalSeparationMeters << " m at "
              << result.minimumVerticalTimeSeconds << " s\n"
              << "Relative motion: "
              << (result.hasRelativeMotion ? "yes" : "no") << '\n'
              << "CPA inside lookahead: "
              << (result.isWithinLookahead ? "yes" : "no") << '\n'
              << "Conflict: " << (result.conflict ? "YES" : "NO") << '\n'
              << "Conflict window: ";
    if (result.conflictWindow.exists) {
        std::cout << result.conflictWindow.startSeconds << " to "
                  << result.conflictWindow.endSeconds << " s\n";
    } else {
        std::cout << "none\n";
    }
    std::cout
              << "Expected: "
              << (scenario.expectedConflict ? "CONFLICT" : "NO CONFLICT")
              << " [" << (result.conflict == scenario.expectedConflict ? "PASS" : "FAIL")
              << "]\n";
}

void printScenarioList() {
    std::cout << "Test Cases\n";
    for (const auto& scenario : ScenarioCatalog::all()) {
        std::cout << "  " << std::left << std::setw(24) << scenario.name
                  << scenario.description << '\n';
    }
    std::cout << std::right
              << "\nRuntime simulations\n"
              << "  " << std::left << std::setw(24) << "random-encounter"
              << "Generate aircraft motion and wind from a new random seed.\n"
              << std::right;
}

void printUsage(const std::string& executable) {
    std::cout << "Usage:\n"
              << "  " << executable << " [scenario]\n"
              << "  " << executable << " --simulate [scenario] [options]\n"
              << "  " << executable << " --list-scenarios\n\n";
    printScenarioList();
    std::cout << "\nCommands:\n"
              << "  " << executable << " all\n"
              << "      Run every scenario as a one-shot check.\n"
              << "  " << executable << " <scenario>\n"
              << "      Run one scenario as a one-shot check.\n"
              << "  " << executable << " --simulate <scenario> [speed]\n"
              << "      Animate one built-in scenario in the terminal.\n"
              << "  " << executable
              << " --simulate random-encounter [speed] [seed]\n"
              << "      Generate and animate a random encounter. A seed reproduces it.\n"
              << "      --prediction deterministic|probabilistic\n"
              << "      --execution sequential|threaded\n"
              << "      --threads N|auto --samples N\n"
              << "      --scenario-seed N --uncertainty-seed N\n"
              << "      --uncertainty low|medium|high\n"
              << "      --outcome any|collision|pass\n\n"
              << "  " << executable
              << " --benchmark [--samples N] [--repetitions N]"
                 " [--threads 1,2,4,auto]\n"
              << "Terminal simulation defaults to head-on at 5x speed.\n";
}

} // namespace

int Application::run(int argc, char* argv[]) const {
    const std::string requestedScenario = argc > 1 ? argv[1] : "all";

    if (requestedScenario == "--help" || requestedScenario == "-h") {
        printUsage(argv[0]);
        return 0;
    }

    if (requestedScenario == "--list-scenarios" ||
        requestedScenario == "--scenarios") {
        printScenarioList();
        return 0;
    }

    if (requestedScenario == "--benchmark") {
        return runPredictionBenchmark(argc, argv);
    }

    const ConflictDetector detector{};

    if (requestedScenario == "--simulate") {
        const CommandLineParseResult parsed = parseSimulationCommand(argc, argv);
        if (!parsed.options.hasValue()) {
            std::cerr << parsed.error << "\n\n";
            printUsage(argv[0]);
            return 2;
        }
        const SimulationCommandOptions& command = *parsed.options;

        const std::string simulationScenario = command.scenarioName;
        const bool randomEncounter = simulationScenario == "random-encounter";
        const Scenario* const scenario = randomEncounter
            ? nullptr
            : ScenarioCatalog::find(simulationScenario);
        if (!randomEncounter && scenario == nullptr) {
            std::cerr << "Unknown simulation scenario: "
                      << simulationScenario << "\n\n";
            printUsage(argv[0]);
            return 2;
        }

        if (!randomEncounter && command.scenarioSeed.hasValue()) {
            std::cerr << "A scenario seed is only valid for random-encounter.\n";
            return 2;
        }
        if (!randomEncounter &&
            command.requestedOutcome != RandomEncounterOutcome::Any) {
            std::cerr << "A requested outcome is only valid for random-encounter.\n";
            return 2;
        }

        const TerminalSimulation simulation{};
        TerminalSimulationOptions options = command.simulation;
        if (randomEncounter) {
            const ScenarioSetup setup = prepareScenario(
                simulationScenario, command.scenarioSeed, command.requestedOutcome);
            options.waypoint = setup.waypoint;
            options.worldBounds = setup.worldBounds;
            options.durationSeconds = setup.durationSeconds;
            options.windVelocity = setup.windVelocity;
            options.randomSeed = setup.seed;
            static_cast<void>(simulation.run(setup.aircraftA, setup.aircraftB, options));
            return 0;
        }

        static_cast<void>(simulation.run(
            scenario->aircraftA,
            scenario->aircraftB,
            options));
        return 0;
    }

    bool foundScenario = false;
    bool allPassed = true;

    for (const auto& scenario : ScenarioCatalog::all()) {
        if (requestedScenario != "all" && requestedScenario != scenario.name) {
            continue;
        }

        foundScenario = true;
        const ClosestApproach result =
            detector.evaluate(scenario.aircraftA, scenario.aircraftB);
        printResult(scenario, result);
        allPassed = allPassed && result.conflict == scenario.expectedConflict;
    }

    if (!foundScenario) {
        std::cerr << "Unknown scenario: " << requestedScenario << "\n\n";
        printUsage(argv[0]);
        return 2;
    }

    return allPassed ? 0 : 1;
}

} // namespace vectorwatch
