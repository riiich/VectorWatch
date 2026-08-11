#include "vectorwatch/app/Application.hpp"

#include "vectorwatch/detection/ConflictDetector.hpp"
#include "vectorwatch/scenarios/ScenarioCatalog.hpp"
#include "vectorwatch/simulation/TerminalSimulation.hpp"

#include <charconv>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string_view>

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
              << "Relative motion: "
              << (result.hasRelativeMotion ? "yes" : "no") << '\n'
              << "CPA inside lookahead: "
              << (result.isWithinLookahead ? "yes" : "no") << '\n'
              << "Conflict: " << (result.conflict ? "YES" : "NO") << '\n'
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
    std::cout << std::right;
}

void printUsage(std::string_view executable) {
    std::cout << "Usage:\n"
              << "  " << executable << " [scenario]\n"
              << "  " << executable << " --simulate [scenario] [speed]\n"
              << "  " << executable << " --list-scenarios\n\n";
    printScenarioList();
    std::cout << "\nCommands:\n"
              << "  " << executable << " all\n"
              << "      Run every scenario as a one-shot check.\n"
              << "  " << executable << " <scenario>\n"
              << "      Run one scenario as a one-shot check.\n"
              << "  " << executable << " --simulate <scenario> [speed]\n"
              << "      Animate one scenario in the terminal.\n\n"
              << "Terminal simulation defaults to head-on at 5x speed.\n";
}

[[nodiscard]] std::optional<double> parsePositiveDouble(std::string_view text) {
    double value = 0.0;
    const char* const end = text.data() + text.size();
    const auto [position, error] = std::from_chars(text.data(), end, value);
    if (error != std::errc{} || position != end || !std::isfinite(value) ||
        value <= 0.0) {
        return std::nullopt;
    }
    return value;
}

} // namespace

int Application::run(int argc, char* argv[]) const {
    const std::string_view requestedScenario = argc > 1 ? argv[1] : "all";

    if (requestedScenario == "--help" || requestedScenario == "-h") {
        printUsage(argv[0]);
        return 0;
    }

    if (requestedScenario == "--list-scenarios" ||
        requestedScenario == "--scenarios") {
        printScenarioList();
        return 0;
    }

    const ConflictDetector detector{};

    if (requestedScenario == "--simulate") {
        if (argc > 4) {
            std::cerr << "Too many arguments for terminal simulation.\n\n";
            printUsage(argv[0]);
            return 2;
        }

        const std::string_view simulationScenario =
            argc > 2 ? argv[2] : "head-on";
        const Scenario* const scenario = ScenarioCatalog::find(simulationScenario);
        if (scenario == nullptr) {
            std::cerr << "Unknown simulation scenario: "
                      << simulationScenario << "\n\n";
            printUsage(argv[0]);
            return 2;
        }

        TerminalSimulationOptions options{};
        if (argc > 3) {
            const std::optional<double> speed = parsePositiveDouble(argv[3]);
            if (!speed.has_value()) {
                std::cerr << "Simulation speed must be a positive number.\n";
                return 2;
            }
            options.speedMultiplier = *speed;
        }

        const TerminalSimulation simulation{detector};
        simulation.run(
            scenario->name,
            scenario->aircraftA,
            scenario->aircraftB,
            options);
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
