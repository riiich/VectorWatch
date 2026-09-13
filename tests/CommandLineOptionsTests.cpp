#include "CommandLineOptionsTests.hpp"

#include "vectorwatch/app/CommandLineOptions.hpp"

#include <iostream>
#include <string>
#include <vector>

namespace {

int failureCount = 0;

void expect(bool condition, const char* testName) {
    if (!condition) {
        std::cerr << "FAIL: " << testName << '\n';
        ++failureCount;
    }
}

vectorwatch::CommandLineParseResult parse(
    std::vector<std::string> arguments) {
    std::vector<char*> argv;
    argv.reserve(arguments.size());
    for (std::string& argument : arguments) {
        argv.push_back(&argument[0]);
    }
    return vectorwatch::parseSimulationCommand(
        static_cast<int>(argv.size()),
        argv.data());
}

} // namespace

int runCommandLineOptionsTests() {
    {
        const auto parsed = parse({
            "vectorwatch",
            "--simulate",
            "random-encounter",
            "--prediction",
            "probabilistic",
            "--execution",
            "threaded",
            "--threads",
            "4",
            "--samples",
            "25000",
            "--scenario-seed",
            "42",
            "--uncertainty-seed",
            "99",
            "--uncertainty",
            "high",
            "--outcome",
            "pass",
        });
        expect(parsed.options.hasValue(), "full simulation command parses");
        if (parsed.options.hasValue()) {
            expect(
                parsed.options->simulation.predictionMode ==
                    vectorwatch::PredictionMode::Probabilistic,
                "probabilistic mode parsed");
            expect(
                parsed.options->simulation.executionMode ==
                    vectorwatch::SimulationExecutionMode::Threaded,
                "threaded execution parsed");
            expect(
                parsed.options->simulation.predictionWorkerCount == 4,
                "worker count parsed");
            expect(
                parsed.options->simulation.predictionSampleCount == 25'000,
                "sample count parsed");
            expect(
                parsed.options->scenarioSeed.hasValue() &&
                    *parsed.options->scenarioSeed == 42U,
                "scenario seed parsed");
        }
    }

    {
        const auto parsed = parse({
            "vectorwatch",
            "--simulate",
            "random-encounter",
            "10",
            "8",
        });
        expect(parsed.options.hasValue(), "legacy speed and seed parse");
        if (parsed.options.hasValue()) {
            expect(
                parsed.options->simulation.speedMultiplier == 10.0,
                "legacy speed retained");
            expect(
                parsed.options->scenarioSeed.hasValue() &&
                    *parsed.options->scenarioSeed == 8U,
                "legacy seed retained");
        }
    }

    expect(
        !parse({
             "vectorwatch",
             "--simulate",
             "head-on",
             "--threads",
             "4",
         }).options.hasValue(),
        "threads rejected for sequential execution");
    expect(
        !parse({
             "vectorwatch",
             "--simulate",
             "head-on",
             "--samples",
             "0",
         }).options.hasValue(),
        "zero samples rejected");

    if (failureCount != 0) {
        std::cerr << failureCount << " command-line option test(s) failed.\n";
    }
    return failureCount;
}
