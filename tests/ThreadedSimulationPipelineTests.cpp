#include "ThreadedSimulationPipelineTests.hpp"

#include "vectorwatch/simulation/ThreadedSimulationPipeline.hpp"

#include <iostream>

namespace {

using vectorwatch::Aircraft;
using vectorwatch::PredictionMode;
using vectorwatch::SimulationEndReason;
using vectorwatch::ThreadedSimulationConfig;
using vectorwatch::ThreadedSimulationPipeline;
using vectorwatch::Vector3;

int failureCount = 0;

void expect(bool condition, const char* testName) {
    if (!condition) {
        std::cerr << "FAIL: " << testName << '\n';
        ++failureCount;
    }
}

} // namespace

int runThreadedSimulationPipelineTests() {
    {
        ThreadedSimulationConfig config;
        config.durationSeconds = 0.02;
        config.speedMultiplier = 100.0;
        config.updateRateHz = 1'000.0;
        config.predictionRateHz = 500.0;
        ThreadedSimulationPipeline pipeline{
            Aircraft{1, Vector3{0.0, 0.0, 10'000.0}, Vector3{10.0, 0.0, 0.0}},
            Aircraft{2, Vector3{0.0, 5'000.0, 10'000.0}, Vector3{10.0, 0.0, 0.0}},
            config};
        const auto finalView = pipeline.waitForFinalView();
        expect(finalView.result.hasValue(), "threaded pipeline completes");
        expect(
            finalView.result->reason == SimulationEndReason::Completed,
            "threaded pipeline reports normal completion");
        expect(
            finalView.prediction.hasValue() &&
                finalView.prediction->snapshotSequence ==
                    finalView.snapshot->sequence,
            "threaded pipeline publishes final matching prediction");
    }

    {
        ThreadedSimulationConfig config;
        config.durationSeconds = 60.0;
        config.speedMultiplier = 1'000'000.0;
        config.updateRateHz = 1'000.0;
        config.predictionRateHz = 500.0;
        config.prediction.mode = PredictionMode::Probabilistic;
        config.prediction.sampleCount = 2'000;
        config.prediction.chunkCount = 32;
        config.prediction.workerCount = 2;
        ThreadedSimulationPipeline pipeline{
            Aircraft{
                1,
                Vector3{-10'000.0, 0.0, 10'000.0},
                Vector3{200.0, 0.0, 0.0}},
            Aircraft{
                2,
                Vector3{10'000.0, 0.0, 10'000.0},
                Vector3{-200.0, 0.0, 0.0}},
            config};
        const auto finalView = pipeline.waitForFinalView();
        expect(
            finalView.result->reason == SimulationEndReason::Collision,
            "threaded pipeline preserves swept collision");
        expect(
            finalView.result->collision.hasValue(),
            "threaded collision exposes collision detail");
    }

    if (failureCount != 0) {
        std::cerr << failureCount << " threaded pipeline test(s) failed.\n";
    }
    return failureCount;
}
