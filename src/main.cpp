#include "vectorwatch/Aircraft.hpp"
#include "vectorwatch/ConflictDetector.hpp"
#include "vectorwatch/TerminalSimulation.hpp"

#include <array>
#include <charconv>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string_view>

namespace {

using vectorwatch::Aircraft;
using vectorwatch::ClosestApproach;
using vectorwatch::ConflictDetector;
using vectorwatch::TerminalSimulationOptions;
using vectorwatch::Vector3;

struct Scenario {
    std::string_view name;
    std::string_view description;
    Aircraft aircraftA;
    Aircraft aircraftB;
    bool expectedConflict;
};

const std::array scenarios{
    Scenario{
        "head-on",
        "Aircraft converge along the same horizontal line.",
        Aircraft{1, Vector3{-10'000.0, 0.0, 10'000.0}, Vector3{200.0, 0.0, 0.0}},
        Aircraft{2, Vector3{10'000.0, 0.0, 10'000.0}, Vector3{-200.0, 0.0, 0.0}},
        true,
    },
    Scenario{
        "parallel",
        "Aircraft maintain equal velocity and constant separation.",
        Aircraft{1, Vector3{0.0, 0.0, 10'000.0}, Vector3{200.0, 0.0, 0.0}},
        Aircraft{2, Vector3{0.0, 5'000.0, 10'000.0}, Vector3{200.0, 0.0, 0.0}},
        false,
    },
    Scenario{
        "crossing",
        "Aircraft reach the same intersection at the same time.",
        Aircraft{1, Vector3{-10'000.0, 0.0, 10'000.0}, Vector3{200.0, 0.0, 0.0}},
        Aircraft{2, Vector3{0.0, -10'000.0, 10'000.0}, Vector3{0.0, 200.0, 0.0}},
        true,
    },
    Scenario{
        "different-altitudes",
        "Horizontal paths cross, but vertical separation remains safe.",
        Aircraft{1, Vector3{-10'000.0, 0.0, 9'000.0}, Vector3{200.0, 0.0, 0.0}},
        Aircraft{2, Vector3{0.0, -10'000.0, 12'000.0}, Vector3{0.0, 200.0, 0.0}},
        false,
    },
};

void printResult(const Scenario& scenario, const ClosestApproach& result) {
    std::cout << "\nScenario: " << scenario.name << '\n'
              << scenario.description << '\n'
              << std::fixed << std::setprecision(2)
              << "Current 3D separation: "
              << result.currentDistanceMeters / 1'000.0 << " km\n"
              << "TCPA (within lookahead): " << result.timeSeconds << " s\n"
              << "Horizontal separation at CPA: "
              << result.horizontalSeparationMeters / 1'000.0 << " km\n"
              << "Vertical separation at CPA: "
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

void printUsage(std::string_view executable) {
    std::cout << "Usage:\n"
              << "  " << executable << " [scenario]\n"
              << "  " << executable
              << " --simulate [scenario] [speed]\n\n"
              << "Scenarios:\n";
    for (const auto& scenario : scenarios) {
        std::cout << "  " << scenario.name << '\n';
    }
    std::cout << "  all (default for one-shot checks)\n\n"
              << "Terminal simulation defaults to head-on at 5x speed.\n";
}

[[nodiscard]] const Scenario* findScenario(std::string_view name) {
    for (const auto& scenario : scenarios) {
        if (scenario.name == name) {
            return &scenario;
        }
    }
    return nullptr;
}

[[nodiscard]] std::optional<double> parsePositiveDouble(std::string_view text) {
    double value = 0.0;
    const char* const end = text.data() + text.size();
    const auto [position, error] =
        std::from_chars(text.data(), end, value);
    if (error != std::errc{} || position != end || !std::isfinite(value) ||
        value <= 0.0) {
        return std::nullopt;
    }
    return value;
}

} // namespace

int main(int argc, char* argv[]) {
    const std::string_view requestedScenario = argc > 1 ? argv[1] : "all";

    if (requestedScenario == "--help" || requestedScenario == "-h") {
        printUsage(argv[0]);
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
        const Scenario* const scenario = findScenario(simulationScenario);
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

        vectorwatch::runTerminalSimulation(
            scenario->name,
            scenario->aircraftA,
            scenario->aircraftB,
            detector,
            options);
        return 0;
    }

    bool foundScenario = false;
    bool allPassed = true;

    for (const auto& scenario : scenarios) {
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
