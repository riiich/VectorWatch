#include "PredictionEngineTests.hpp"

#include "vectorwatch/prediction/PredictionEngine.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

using vectorwatch::Aircraft;
using vectorwatch::PredictionEngineConfig;
using vectorwatch::PredictionMode;
using vectorwatch::RiskLevel;
using vectorwatch::StateSnapshot;
using vectorwatch::UncertaintyConfig;
using vectorwatch::Vector3;

int failureCount = 0;

void expect(bool condition, const char* testName) {
    if (!condition) {
        std::cerr << "FAIL: " << testName << '\n';
        ++failureCount;
    }
}

StateSnapshot headOnSnapshot() {
    return StateSnapshot(
        42,
        5.0,
        Aircraft{
            1,
            Vector3{-10'000.0, 0.0, 10'000.0},
            Vector3{200.0, 0.0, 0.0}},
        Aircraft{
            2,
            Vector3{10'000.0, 0.0, 10'000.0},
            Vector3{-200.0, 0.0, 0.0}});
}

UncertaintyConfig zeroUncertainty() {
    return UncertaintyConfig(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

} // namespace

int runPredictionEngineTests() {
    {
        auto engine = vectorwatch::makePredictionEngine({});
        const auto result = engine->predict(headOnSnapshot(), {});
        expect(result.snapshotSequence == 42, "prediction preserves sequence");
        expect(result.nominalApproach.conflict, "deterministic conflict detected");
        expect(result.nominalCollision.hasValue(), "deterministic collision detected");
        expect(
            result.riskLevel == RiskLevel::HighlyLikely,
            "deterministic conflict maps to high risk");
    }

    {
        PredictionEngineConfig config;
        config.mode = PredictionMode::Probabilistic;
        config.sampleCount = 1'000;
        config.uncertaintySeed = 77;
        config.chunkCount = 32;
        config.uncertainty = zeroUncertainty();
        auto engine = vectorwatch::makePredictionEngine(config);
        const StateSnapshot snapshot = headOnSnapshot();
        const Vector3 originalPosition = snapshot.aircraftA.position();
        const auto result = engine->predict(snapshot, {});
        expect(result.conflictCount == 1'000, "all zero-noise samples conflict");
        expect(result.collisionCount == 1'000, "all zero-noise samples collide");
        expect(
            snapshot.aircraftA.position().x == originalPosition.x,
            "sampling does not mutate nominal snapshot");
    }

    {
        PredictionEngineConfig config;
        config.mode = PredictionMode::Probabilistic;
        config.sampleCount = 2'000;
        config.uncertaintySeed = 1234;
        config.chunkCount = 64;
        auto firstEngine = vectorwatch::makePredictionEngine(config);
        auto secondEngine = vectorwatch::makePredictionEngine(config);
        const auto first = firstEngine->predict(headOnSnapshot(), {});
        const auto second = secondEngine->predict(headOnSnapshot(), {});
        expect(
            first.conflictCount == second.conflictCount,
            "uncertainty seed reproduces conflict count");
        expect(
            first.collisionCount == second.collisionCount,
            "uncertainty seed reproduces collision count");

        config.workerCount = 4;
        auto threadedEngine = vectorwatch::makePredictionEngine(config);
        const auto threaded = threadedEngine->predict(headOnSnapshot(), {});
        expect(
            threaded.conflictCount == first.conflictCount,
            "threaded prediction preserves conflict count");
        expect(
            threaded.collisionCount == first.collisionCount,
            "threaded prediction preserves collision count");
        expect(threaded.workerCount == 4, "threaded result reports workers");
    }

    expect(
        vectorwatch::classifyRisk(0.049) == RiskLevel::Low,
        "probability below five percent is low risk");
    expect(
        vectorwatch::classifyRisk(0.05) == RiskLevel::Potential,
        "five percent enters potential conflict");
    expect(
        vectorwatch::classifyRisk(0.70) == RiskLevel::HighlyLikely,
        "seventy percent enters highly likely conflict");

    bool zeroSamplesRejected = false;
    try {
        PredictionEngineConfig invalidConfig;
        invalidConfig.sampleCount = 0;
        static_cast<void>(vectorwatch::makePredictionEngine(invalidConfig));
    } catch (const std::invalid_argument&) {
        zeroSamplesRejected = true;
    }
    expect(zeroSamplesRejected, "zero prediction samples rejected");

    if (failureCount != 0) {
        std::cerr << failureCount << " prediction engine test(s) failed.\n";
    }
    return failureCount;
}
