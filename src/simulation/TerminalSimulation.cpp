#include "vectorwatch/simulation/TerminalSimulation.hpp"

#include "vectorwatch/simulation/TerminalRadarRenderer.hpp"
#include "vectorwatch/simulation/ThreadedSimulationPipeline.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <thread>
#include <utility>

namespace vectorwatch {
namespace {

std::size_t resolvedPredictionWorkerCount(
    const TerminalSimulationOptions& options) {
    if (options.executionMode == SimulationExecutionMode::Sequential) {
        return 0;
    }
    if (options.predictionWorkerCount > 0) {
        return options.predictionWorkerCount;
    }
    const unsigned int available = std::thread::hardware_concurrency();
    return available > 2U
        ? static_cast<std::size_t>(available - 2U)
        : 1U;
}

PredictionEngineConfig makePredictionConfig(
    const TerminalSimulationOptions& options) {
    PredictionEngineConfig config;
    config.mode = options.predictionMode;
    config.sampleCount = options.predictionSampleCount;
    config.uncertaintySeed = options.uncertaintySeed;
    config.workerCount = resolvedPredictionWorkerCount(options);
    config.uncertainty = uncertaintyConfigFor(options.uncertaintyProfile);
    return config;
}

SimulationResult runThreaded(
    Aircraft aircraftA,
    Aircraft aircraftB,
    const TerminalSimulationOptions& options,
    PredictionEngineConfig predictionConfig) {
    const TerminalRadarRenderer renderer{
        aircraftA,
        aircraftB,
        options};
    renderer.prepareTerminal();

    ThreadedSimulationConfig threadedConfig;
    threadedConfig.durationSeconds = options.durationSeconds;
    threadedConfig.speedMultiplier = options.speedMultiplier;
    threadedConfig.updateRateHz = options.updateRateHz;
    threadedConfig.predictionRateHz = options.predictionRateHz;
    threadedConfig.windVelocity = options.windVelocity;
    threadedConfig.worldBounds = options.worldBounds;
    threadedConfig.prediction = std::move(predictionConfig);

    ThreadedSimulationPipeline pipeline(
        std::move(aircraftA),
        std::move(aircraftB),
        std::move(threadedConfig));

    using Clock = std::chrono::steady_clock;
    const auto frameInterval = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>{1.0 / options.updateRateHz});
    auto nextFrame = Clock::now();

    while (true) {
        const ThreadedSimulationView view = pipeline.view();
        if (view.result.hasValue() &&
            view.result->reason == SimulationEndReason::Collision) {
            const StateSnapshot& snapshot = *view.snapshot;
            const Collision& collision = *view.result->collision;
            renderer.animateCollision(
                snapshot.aircraftA,
                snapshot.aircraftB,
                collision.position,
                snapshot.simulationTimeSeconds,
                options);
            renderer.finish(snapshot, view.prediction, *view.result);
            return *view.result;
        }

        if (view.snapshot.hasValue() && view.prediction.hasValue()) {
            renderer.render(
                view.snapshot->aircraftA,
                view.snapshot->aircraftB,
                *view.prediction,
                options);
        }

        if (view.result.hasValue()) {
            if (view.result->reason == SimulationEndReason::Cancelled ||
                (view.prediction.hasValue() && view.snapshot.hasValue() &&
                 view.prediction->snapshotSequence == view.snapshot->sequence)) {
                renderer.finish(*view.snapshot, view.prediction, *view.result);
                return *view.result;
            }
        }

        nextFrame += frameInterval;
        std::this_thread::sleep_until(nextFrame);
    }
}

} // namespace

SimulationResult TerminalSimulation::run(
    Aircraft aircraftA,
    Aircraft aircraftB,
    TerminalSimulationOptions options) const {
    if (options.speedMultiplier <= 0.0 || options.durationSeconds <= 0.0 ||
        options.updateRateHz <= 0.0 || options.predictionRateHz <= 0.0) {
        throw std::invalid_argument{"Simulation options must be positive"};
    }

    PredictionEngineConfig predictionConfig = makePredictionConfig(options);
    if (options.executionMode == SimulationExecutionMode::Threaded) {
        return runThreaded(
            std::move(aircraftA),
            std::move(aircraftB),
            options,
            std::move(predictionConfig));
    }

    std::unique_ptr<PredictionEngine> predictionEngine =
        makePredictionEngine(predictionConfig);

    const TerminalRadarRenderer renderer{
        aircraftA,
        aircraftB,
        options};
    renderer.prepareTerminal();

    using Clock = std::chrono::steady_clock;
    const auto frameInterval = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>{1.0 / options.updateRateHz});
    const auto predictionInterval = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>{1.0 / options.predictionRateHz});
    auto previousFrame = Clock::now();
    auto nextFrame = previousFrame;
    auto nextPrediction = previousFrame;
    SimulationSession session{
        std::move(aircraftA),
        std::move(aircraftB),
        options.durationSeconds,
        options.worldBounds};
    bool firstFrame = true;
    Optional<PredictionResult> latestPrediction{};

    while (true) {
        const StateSnapshot& snapshot = session.snapshot();
        if (firstFrame) {
            if (session.result().hasValue()) {
                const Collision& collision = *session.result()->collision;
                renderer.animateCollision(
                    snapshot.aircraftA,
                    snapshot.aircraftB,
                    collision.position,
                    snapshot.simulationTimeSeconds,
                    options);
                renderer.finish(snapshot, latestPrediction, *session.result());
                return *session.result();
            }
        } else {
            const auto currentFrame = Clock::now();
            const double realDeltaSeconds =
                std::chrono::duration<double>{currentFrame - previousFrame}.count();
            const double simulationDelta = std::min(
                realDeltaSeconds * options.speedMultiplier,
                options.durationSeconds - snapshot.simulationTimeSeconds);

            session.advance(simulationDelta);
            const StateSnapshot& advancedSnapshot = session.snapshot();
            if (session.result().hasValue() &&
                session.result()->reason == SimulationEndReason::Collision) {
                const Collision& collision = *session.result()->collision;
                renderer.animateCollision(
                    advancedSnapshot.aircraftA,
                    advancedSnapshot.aircraftB,
                    collision.position,
                    advancedSnapshot.simulationTimeSeconds,
                    options);
                renderer.finish(
                    advancedSnapshot,
                    latestPrediction,
                    *session.result());
                return *session.result();
            }
            previousFrame = currentFrame;
        }

        const StateSnapshot& renderedSnapshot = session.snapshot();
        const auto predictionTime = Clock::now();
        if (!latestPrediction.hasValue() ||
            predictionTime >= nextPrediction ||
            session.result().hasValue()) {
            latestPrediction = predictionEngine->predict(
                renderedSnapshot,
                options.windVelocity);
            nextPrediction = predictionTime + predictionInterval;
        }
        renderer.render(
            renderedSnapshot.aircraftA,
            renderedSnapshot.aircraftB,
            *latestPrediction,
            options);

        if (session.result().hasValue()) {
            break;
        }

        nextFrame += frameInterval;
        std::this_thread::sleep_until(nextFrame);
        firstFrame = false;
    }

    renderer.finish(session.snapshot(), latestPrediction, *session.result());
    return *session.result();
}

} // namespace vectorwatch
