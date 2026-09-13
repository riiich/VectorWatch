#pragma once

#include "vectorwatch/prediction/PredictionEngine.hpp"
#include "vectorwatch/simulation/SimulationSession.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <thread>

namespace vectorwatch {

struct ThreadedSimulationConfig {
    double durationSeconds{60.0};
    double speedMultiplier{5.0};
    double updateRateHz{20.0};
    double predictionRateHz{5.0};
    Vector3 windVelocity{};
    Optional<WorldBounds> worldBounds{};
    PredictionEngineConfig prediction{};
};

struct ThreadedSimulationView {
    Optional<StateSnapshot> snapshot{};
    Optional<PredictionResult> prediction{};
    Optional<SimulationResult> result{};
};

// Runs state updates and prediction coordination on separate threads. Callers
// receive copies through view(), so rendering never changes aircraft state.
class ThreadedSimulationPipeline {
public:
    ThreadedSimulationPipeline(
        Aircraft aircraftA,
        Aircraft aircraftB,
        ThreadedSimulationConfig config);
    ~ThreadedSimulationPipeline();

    ThreadedSimulationPipeline(const ThreadedSimulationPipeline&) = delete;
    ThreadedSimulationPipeline& operator=(const ThreadedSimulationPipeline&) = delete;
    ThreadedSimulationPipeline(ThreadedSimulationPipeline&&) = delete;
    ThreadedSimulationPipeline& operator=(ThreadedSimulationPipeline&&) = delete;

    ThreadedSimulationView view() const;
    ThreadedSimulationView waitForFinalView();
    void requestStop() noexcept;

private:
    Aircraft initialAircraftA_;
    Aircraft initialAircraftB_;
    ThreadedSimulationConfig config_;

    mutable std::mutex mutex_{};
    std::condition_variable stateChanged_{};
    Optional<StateSnapshot> latestSnapshot_{};
    Optional<StateSnapshot> pendingPrediction_{};
    Optional<PredictionResult> latestPrediction_{};
    Optional<SimulationResult> result_{};
    std::exception_ptr failure_{};
    bool stopping_{};
    std::atomic<bool> stopRequested_{};
    std::atomic<std::uint64_t> predictionRequestVersion_{};

    std::unique_ptr<PredictionEngine> predictionEngine_;
    std::thread simulationThread_{};
    std::thread predictionThread_{};

    void runSimulation();
    void runPrediction();
    void publishSnapshot(
        const StateSnapshot& snapshot,
        bool requestPrediction,
        const Optional<SimulationResult>& result = Optional<SimulationResult>());
    void recordFailure() noexcept;
};

} // namespace vectorwatch
