#include "vectorwatch/app/BenchmarkRunner.hpp"

#include "vectorwatch/prediction/PredictionEngine.hpp"
#include "vectorwatch/scenarios/RandomEncounterGenerator.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace vectorwatch {
namespace {

struct BenchmarkOptions {
    std::size_t sampleCount{1'000'000};
    std::size_t repetitions{3};
    std::uint32_t scenarioSeed{42};
    std::uint32_t uncertaintySeed{1};
    std::vector<std::size_t> workerCounts{1, 2, 4};
    bool includeAutomaticWorkers{true};
};

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

std::size_t automaticWorkerCount() noexcept {
    const unsigned int available = std::thread::hardware_concurrency();
    return available > 2U
        ? static_cast<std::size_t>(available - 2U)
        : 1U;
}

bool parseWorkerList(
    const std::string& text,
    BenchmarkOptions& options) {
    options.workerCounts.clear();
    options.includeAutomaticWorkers = false;
    std::size_t start = 0;
    while (start <= text.size()) {
        const std::size_t comma = text.find(',', start);
        const std::string item = text.substr(
            start,
            comma == std::string::npos ? text.size() - start : comma - start);
        if (item == "auto") {
            options.includeAutomaticWorkers = true;
        } else {
            const auto workers = parseInteger<std::size_t>(item);
            if (!workers.hasValue() || *workers == 0) {
                return false;
            }
            options.workerCounts.push_back(*workers);
        }
        if (comma == std::string::npos) {
            break;
        }
        start = comma + 1;
    }
    return !options.workerCounts.empty() || options.includeAutomaticWorkers;
}

Optional<BenchmarkOptions> parseOptions(
    int argc,
    char* argv[]) {
    BenchmarkOptions options{};
    for (int index = 2; index < argc; index += 2) {
        if (index + 1 >= argc) {
            return Optional<BenchmarkOptions>();
        }
        const std::string flag = argv[index];
        const std::string value = argv[index + 1];
        if (flag == "--samples") {
            const auto parsed = parseInteger<std::size_t>(value);
            if (!parsed.hasValue() || *parsed == 0) {
                return Optional<BenchmarkOptions>();
            }
            options.sampleCount = *parsed;
        } else if (flag == "--repetitions") {
            const auto parsed = parseInteger<std::size_t>(value);
            if (!parsed.hasValue() || *parsed == 0) {
                return Optional<BenchmarkOptions>();
            }
            options.repetitions = *parsed;
        } else if (flag == "--scenario-seed") {
            const auto parsed = parseInteger<std::uint32_t>(value);
            if (!parsed.hasValue()) {
                return Optional<BenchmarkOptions>();
            }
            options.scenarioSeed = *parsed;
        } else if (flag == "--uncertainty-seed") {
            const auto parsed = parseInteger<std::uint32_t>(value);
            if (!parsed.hasValue()) {
                return Optional<BenchmarkOptions>();
            }
            options.uncertaintySeed = *parsed;
        } else if (flag == "--threads") {
            if (!parseWorkerList(value, options)) {
                return Optional<BenchmarkOptions>();
            }
        } else {
            return Optional<BenchmarkOptions>();
        }
    }
    return options;
}

struct Measurement {
    std::size_t workers{};
    double medianMilliseconds{};
    std::size_t conflictCount{};
    std::size_t collisionCount{};
};

Measurement measure(
    const StateSnapshot& snapshot,
    const Vector3& windVelocity,
    const BenchmarkOptions& options,
    std::size_t workerCount) {
    PredictionEngineConfig config;
    config.mode = PredictionMode::Probabilistic;
    config.sampleCount = options.sampleCount;
    config.uncertaintySeed = options.uncertaintySeed;
    config.workerCount = workerCount;
    config.uncertainty = uncertaintyConfigFor(UncertaintyProfile::Medium);
    std::unique_ptr<PredictionEngine> engine = makePredictionEngine(config);
    std::vector<double> durations;
    durations.reserve(options.repetitions);
    PredictionResult last = engine->predict(snapshot, windVelocity);
    durations.push_back(last.calculationMilliseconds);
    for (std::size_t repetition = 1; repetition < options.repetitions; ++repetition) {
        last = engine->predict(snapshot, windVelocity);
        durations.push_back(last.calculationMilliseconds);
    }
    std::sort(durations.begin(), durations.end());
    Measurement measurement;
    measurement.workers = workerCount;
    measurement.medianMilliseconds = durations[durations.size() / 2];
    measurement.conflictCount = last.conflictCount;
    measurement.collisionCount = last.collisionCount;
    return measurement;
}

} // namespace

int runPredictionBenchmark(int argc, char* argv[]) {
    const Optional<BenchmarkOptions> parsed = parseOptions(argc, argv);
    if (!parsed.hasValue()) {
        std::cerr << "Invalid benchmark options. Use --samples N, --repetitions N, "
                     "--threads 1,2,4,auto, --scenario-seed N, or "
                     "--uncertainty-seed N.\n";
        return 2;
    }
    BenchmarkOptions options = *parsed;
    if (options.includeAutomaticWorkers) {
        options.workerCounts.push_back(automaticWorkerCount());
    }
    std::sort(options.workerCounts.begin(), options.workerCounts.end());
    options.workerCounts.erase(
        std::unique(options.workerCounts.begin(), options.workerCounts.end()),
        options.workerCounts.end());

    const RandomEncounter encounter = RandomEncounterGenerator::generate(
        options.scenarioSeed,
        RandomEncounterOutcome::Pass);
    const StateSnapshot snapshot(
        0,
        0.0,
        encounter.aircraftA,
        encounter.aircraftB);

    const Measurement baseline = measure(
        snapshot,
        encounter.windVelocity,
        options,
        0);
    std::cout << "VectorWatch probabilistic prediction benchmark\n"
              << "Samples: " << options.sampleCount
              << "  Repetitions: " << options.repetitions << '\n'
              << std::fixed << std::setprecision(2)
              << "sequential  " << baseline.medianMilliseconds << " ms  "
              << (static_cast<double>(options.sampleCount) * 1'000.0 /
                  baseline.medianMilliseconds)
              << " samples/s  speedup 1.00x\n";

    for (const std::size_t workers : options.workerCounts) {
        const Measurement current = measure(
            snapshot,
            encounter.windVelocity,
            options,
            workers);
        if (current.conflictCount != baseline.conflictCount ||
            current.collisionCount != baseline.collisionCount) {
            std::cerr << "Benchmark result mismatch for " << workers
                      << " worker(s).\n";
            return 1;
        }
        std::cout << "workers " << std::setw(3) << workers << "  "
                  << current.medianMilliseconds << " ms  "
                  << (static_cast<double>(options.sampleCount) * 1'000.0 /
                      current.medianMilliseconds)
                  << " samples/s  speedup "
                  << baseline.medianMilliseconds / current.medianMilliseconds
                  << "x\n";
    }

    std::cout << "All worker configurations verified identical counts.\n";
    return 0;
}

} // namespace vectorwatch
