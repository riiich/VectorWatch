#include "vectorwatch/simulation/TerminalSimulation.hpp"

#include "vectorwatch/simulation/TerminalRadarRenderer.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <thread>

namespace vectorwatch {

TerminalSimulation::TerminalSimulation(const ConflictDetector& detector) noexcept
    : detector_{detector} {}

void TerminalSimulation::run(
    std::string_view scenarioName,
    Aircraft aircraftA,
    Aircraft aircraftB,
    TerminalSimulationOptions options) const {
    if (options.speedMultiplier <= 0.0 || options.durationSeconds <= 0.0 ||
        options.updateRateHz <= 0.0) {
        throw std::invalid_argument{"Simulation options must be positive"};
    }

    const TerminalRadarRenderer renderer{
        aircraftA,
        aircraftB,
        options.durationSeconds};
    renderer.prepareTerminal();

    using Clock = std::chrono::steady_clock;
    const auto frameInterval = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>{1.0 / options.updateRateHz});
    auto previousFrame = Clock::now();
    auto nextFrame = previousFrame;
    double simulationTime = 0.0;
    bool firstFrame = true;

    while (true) {
        if (firstFrame) {
            const auto collision =
                collisionDetector_.predict(aircraftA, aircraftB, 0.0);
            if (collision.has_value()) {
                renderer.animateCollision(
                    scenarioName,
                    aircraftA,
                    aircraftB,
                    collision->position,
                    simulationTime);
                return;
            }
        } else {
            const auto currentFrame = Clock::now();
            const double realDeltaSeconds =
                std::chrono::duration<double>{currentFrame - previousFrame}.count();
            const double simulationDelta = std::min(
                realDeltaSeconds * options.speedMultiplier,
                options.durationSeconds - simulationTime);

            const auto collision = collisionDetector_.predict(
                aircraftA,
                aircraftB,
                simulationDelta);
            if (collision.has_value()) {
                aircraftA.update(collision->timeSeconds);
                aircraftB.update(collision->timeSeconds);
                simulationTime += collision->timeSeconds;
                renderer.animateCollision(
                    scenarioName,
                    aircraftA,
                    aircraftB,
                    collision->position,
                    simulationTime);
                return;
            }

            aircraftA.update(simulationDelta);
            aircraftB.update(simulationDelta);
            simulationTime += simulationDelta;
            previousFrame = currentFrame;
        }

        const ClosestApproach approach = detector_.evaluate(aircraftA, aircraftB);
        renderer.render(
            scenarioName,
            aircraftA,
            aircraftB,
            approach,
            options,
            simulationTime);

        if (simulationTime >= options.durationSeconds) {
            break;
        }

        nextFrame += frameInterval;
        std::this_thread::sleep_until(nextFrame);
        firstFrame = false;
    }

    renderer.finish();
}

} // namespace vectorwatch
