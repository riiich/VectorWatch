#include "vectorwatch/simulation/ThreadedSimulationPipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace vectorwatch {

ThreadedSimulationPipeline::ThreadedSimulationPipeline(
    Aircraft aircraftA,
    Aircraft aircraftB,
    ThreadedSimulationConfig config)
    : initialAircraftA_{std::move(aircraftA)},
      initialAircraftB_{std::move(aircraftB)},
      config_{std::move(config)},
      predictionEngine_{makePredictionEngine(config_.prediction)} {
    if (!std::isfinite(config_.durationSeconds) ||
        !std::isfinite(config_.speedMultiplier) ||
        !std::isfinite(config_.updateRateHz) ||
        !std::isfinite(config_.predictionRateHz) ||
        config_.durationSeconds <= 0.0 || config_.speedMultiplier <= 0.0 ||
        config_.updateRateHz <= 0.0 || config_.predictionRateHz <= 0.0) {
        throw std::invalid_argument{
            "Threaded simulation timing values must be finite and positive"};
    }

    try {
        simulationThread_ = std::thread{[this] { runSimulation(); }};
        predictionThread_ = std::thread{[this] { runPrediction(); }};
    } catch (...) {
        requestStop();
        if (simulationThread_.joinable()) {
            simulationThread_.join();
        }
        if (predictionThread_.joinable()) {
            predictionThread_.join();
        }
        throw;
    }
}

ThreadedSimulationPipeline::~ThreadedSimulationPipeline() {
    requestStop();
    if (simulationThread_.joinable()) {
        simulationThread_.join();
    }
    if (predictionThread_.joinable()) {
        predictionThread_.join();
    }
}

ThreadedSimulationView ThreadedSimulationPipeline::view() const {
    const std::lock_guard<std::mutex> lock{mutex_};
    if (failure_) {
        std::rethrow_exception(failure_);
    }
    ThreadedSimulationView currentView;
    currentView.snapshot = latestSnapshot_;
    currentView.prediction = latestPrediction_;
    currentView.result = result_;
    return currentView;
}

ThreadedSimulationView ThreadedSimulationPipeline::waitForFinalView() {
    std::unique_lock<std::mutex> lock{mutex_};
    stateChanged_.wait(lock, [this] {
        if (failure_) {
            return true;
        }
        if (!result_.hasValue() || !latestSnapshot_.hasValue()) {
            return false;
        }
        if (result_->reason != SimulationEndReason::Completed) {
            return true;
        }
        return latestPrediction_.hasValue() &&
            latestPrediction_->snapshotSequence == latestSnapshot_->sequence;
    });
    if (failure_) {
        std::rethrow_exception(failure_);
    }
    ThreadedSimulationView finalView;
    finalView.snapshot = latestSnapshot_;
    finalView.prediction = latestPrediction_;
    finalView.result = result_;
    return finalView;
}

void ThreadedSimulationPipeline::requestStop() noexcept {
    stopRequested_.store(true, std::memory_order_relaxed);
    {
        const std::lock_guard<std::mutex> lock{mutex_};
        stopping_ = true;
    }
    predictionRequestVersion_.fetch_add(1, std::memory_order_relaxed);
    stateChanged_.notify_all();
}

void ThreadedSimulationPipeline::runSimulation() {
    try {
        SimulationSession session{
            initialAircraftA_,
            initialAircraftB_,
            config_.durationSeconds,
            config_.worldBounds};
        publishSnapshot(session.snapshot(), true, session.result());
        if (session.result().hasValue()) {
            return;
        }

        using Clock = std::chrono::steady_clock;
        const auto frameInterval = std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>{1.0 / config_.updateRateHz});
        const auto predictionInterval =
            std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>{1.0 / config_.predictionRateHz});
        auto previousFrame = Clock::now();
        auto nextFrame = previousFrame + frameInterval;
        auto nextPrediction = previousFrame + predictionInterval;

        while (!stopRequested_.load(std::memory_order_relaxed)) {
            std::this_thread::sleep_until(nextFrame);
            if (stopRequested_.load(std::memory_order_relaxed)) {
                break;
            }
            const auto currentFrame = Clock::now();
            const double realDeltaSeconds =
                std::chrono::duration<double>{currentFrame - previousFrame}.count();
            session.advance(realDeltaSeconds * config_.speedMultiplier);
            previousFrame = currentFrame;
            nextFrame += frameInterval;

            const bool finished = session.result().hasValue();
            const bool predictionDue = currentFrame >= nextPrediction || finished;
            if (predictionDue) {
                nextPrediction = currentFrame + predictionInterval;
            }
            publishSnapshot(
                session.snapshot(),
                predictionDue,
                session.result());
            if (finished) {
                return;
            }
        }

        session.cancel();
        publishSnapshot(session.snapshot(), false, session.result());
    } catch (...) {
        recordFailure();
    }
}

void ThreadedSimulationPipeline::runPrediction() {
    try {
        std::uint64_t consumedVersion = 0;
        while (!stopRequested_.load(std::memory_order_relaxed)) {
            Optional<StateSnapshot> snapshot{};
            std::uint64_t requestVersion = 0;
            bool simulationFinished = false;
            {
                std::unique_lock<std::mutex> lock{mutex_};
                stateChanged_.wait(
                    lock,
                    [this, consumedVersion] {
                        return stopping_ || failure_ ||
                            (pendingPrediction_.hasValue() &&
                             predictionRequestVersion_.load(
                                 std::memory_order_relaxed) > consumedVersion) ||
                            result_.hasValue();
                    });
                if (stopping_ || failure_ ||
                    stopRequested_.load(std::memory_order_relaxed)) {
                    return;
                }
                requestVersion = predictionRequestVersion_.load(
                    std::memory_order_relaxed);
                if (!pendingPrediction_.hasValue() ||
                    requestVersion <= consumedVersion) {
                    if (result_.hasValue()) {
                        return;
                    }
                    continue;
                }
                snapshot = pendingPrediction_;
                simulationFinished = result_.hasValue();
            }

            Optional<PredictionResult> prediction =
                predictionEngine_->tryPredict(
                    *snapshot,
                    config_.windVelocity,
                    [this, requestVersion] {
                        return stopRequested_.load(std::memory_order_relaxed) ||
                            predictionRequestVersion_.load(
                                std::memory_order_relaxed) != requestVersion;
                    });
            consumedVersion = requestVersion;

            if (prediction.hasValue()) {
                const std::lock_guard<std::mutex> lock{mutex_};
                if (predictionRequestVersion_.load(std::memory_order_relaxed) ==
                    requestVersion) {
                    latestPrediction_ = std::move(prediction);
                }
            }
            stateChanged_.notify_all();
            if (simulationFinished &&
                predictionRequestVersion_.load(std::memory_order_relaxed) ==
                    requestVersion) {
                return;
            }
        }
    } catch (...) {
        recordFailure();
    }
}

void ThreadedSimulationPipeline::publishSnapshot(
    const StateSnapshot& snapshot,
    bool requestPrediction,
    const Optional<SimulationResult>& result) {
    {
        const std::lock_guard<std::mutex> lock{mutex_};
        latestSnapshot_ = snapshot;
        if (requestPrediction) {
            pendingPrediction_ = snapshot;
            predictionRequestVersion_.fetch_add(1, std::memory_order_relaxed);
        }
        if (result.hasValue()) {
            result_ = result;
        }
    }
    stateChanged_.notify_all();
}

void ThreadedSimulationPipeline::recordFailure() noexcept {
    stopRequested_.store(true, std::memory_order_relaxed);
    {
        const std::lock_guard<std::mutex> lock{mutex_};
        if (!failure_) {
            failure_ = std::current_exception();
        }
        stopping_ = true;
    }
    predictionRequestVersion_.fetch_add(1, std::memory_order_relaxed);
    stateChanged_.notify_all();
}

} // namespace vectorwatch
