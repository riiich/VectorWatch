#include "vectorwatch/TerminalSimulation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

namespace vectorwatch {
namespace {

constexpr int radarWidth = 61;
constexpr int radarHeight = 21;

struct Bounds {
    double minimumX;
    double maximumX;
    double minimumY;
    double maximumY;
};

struct GridPoint {
    int x;
    int y;
};

[[nodiscard]] Bounds calculateBounds(
    const Aircraft& aircraftA,
    const Aircraft& aircraftB,
    double durationSeconds) {
    const Vector3 endA =
        aircraftA.position() + (aircraftA.velocity() * durationSeconds);
    const Vector3 endB =
        aircraftB.position() + (aircraftB.velocity() * durationSeconds);

    double minimumX = std::min(
        {aircraftA.position().x, aircraftB.position().x, endA.x, endB.x});
    double maximumX = std::max(
        {aircraftA.position().x, aircraftB.position().x, endA.x, endB.x});
    double minimumY = std::min(
        {aircraftA.position().y, aircraftB.position().y, endA.y, endB.y});
    double maximumY = std::max(
        {aircraftA.position().y, aircraftB.position().y, endA.y, endB.y});

    const double xPadding = std::max((maximumX - minimumX) * 0.1, 1'000.0);
    const double yPadding = std::max((maximumY - minimumY) * 0.1, 1'000.0);

    minimumX -= xPadding;
    maximumX += xPadding;
    minimumY -= yPadding;
    maximumY += yPadding;

    return {minimumX, maximumX, minimumY, maximumY};
}

[[nodiscard]] GridPoint toGrid(const Vector3& position, const Bounds& bounds) {
    const double normalizedX =
        (position.x - bounds.minimumX) / (bounds.maximumX - bounds.minimumX);
    const double normalizedY =
        (position.y - bounds.minimumY) / (bounds.maximumY - bounds.minimumY);

    const int x = std::clamp(
        static_cast<int>(std::lround(normalizedX * (radarWidth - 1))),
        0,
        radarWidth - 1);
    const int y = std::clamp(
        radarHeight - 1 -
            static_cast<int>(std::lround(normalizedY * (radarHeight - 1))),
        0,
        radarHeight - 1);
    return {x, y};
}

void drawPath(
    std::vector<std::string>& radar,
    GridPoint start,
    GridPoint end) {
    const int deltaX = end.x - start.x;
    const int deltaY = end.y - start.y;
    const int steps = std::max(std::abs(deltaX), std::abs(deltaY));

    if (steps == 0) {
        return;
    }

    for (int step = 0; step <= steps; ++step) {
        const double progress =
            static_cast<double>(step) / static_cast<double>(steps);
        const int x = static_cast<int>(
            std::lround(start.x + (static_cast<double>(deltaX) * progress)));
        const int y = static_cast<int>(
            std::lround(start.y + (static_cast<double>(deltaY) * progress)));
        if (radar[y][x] == ' ') {
            radar[y][x] = '.';
        }
    }
}

void drawMarker(
    std::vector<std::string>& radar,
    GridPoint position,
    char marker) {
    char& cell = radar[position.y][position.x];
    if ((cell == 'A' || cell == 'B' || cell == '*') && marker != 'X') {
        cell = '*';
    } else {
        cell = marker;
    }
}

[[nodiscard]] std::string conflictStatus(const ClosestApproach& approach) {
    if (!approach.hasRelativeMotion) {
        return "NO RELATIVE MOTION";
    }
    if (!approach.isWithinLookahead) {
        return "NO FUTURE CONFLICT";
    }
    return approach.conflict ? "PREDICTED CONFLICT" : "CLEAR";
}

void renderFrame(
    std::string_view scenarioName,
    const Aircraft& aircraftA,
    const Aircraft& aircraftB,
    const ClosestApproach& approach,
    const Bounds& bounds,
    const TerminalSimulationOptions& options,
    double simulationTime,
    bool redrawInPlace) {
    std::vector<std::string> radar(
        radarHeight,
        std::string(static_cast<std::size_t>(radarWidth), ' '));

    double projectionSeconds = 15.0;
    if (approach.hasRelativeMotion && approach.isWithinLookahead &&
        approach.timeSeconds > 0.0) {
        projectionSeconds = approach.timeSeconds;
    }

    const Vector3 projectedA =
        aircraftA.position() + (aircraftA.velocity() * projectionSeconds);
    const Vector3 projectedB =
        aircraftB.position() + (aircraftB.velocity() * projectionSeconds);
    drawPath(
        radar,
        toGrid(aircraftA.position(), bounds),
        toGrid(projectedA, bounds));
    drawPath(
        radar,
        toGrid(aircraftB.position(), bounds),
        toGrid(projectedB, bounds));

    if (approach.hasRelativeMotion && approach.isWithinLookahead) {
        const Vector3 cpaA =
            aircraftA.position() + (aircraftA.velocity() * approach.timeSeconds);
        const Vector3 cpaB =
            aircraftB.position() + (aircraftB.velocity() * approach.timeSeconds);
        drawMarker(radar, toGrid((cpaA + cpaB) * 0.5, bounds), 'X');
    }

    drawMarker(radar, toGrid(aircraftA.position(), bounds), 'A');
    drawMarker(radar, toGrid(aircraftB.position(), bounds), 'B');

    const Vector3 currentSeparation =
        aircraftB.position() - aircraftA.position();
    const double currentHorizontal =
        std::hypot(currentSeparation.x, currentSeparation.y);
    const double currentVertical = std::abs(currentSeparation.z);

    std::ostringstream frame;
    frame << std::fixed << std::setprecision(1)
          << "VECTORWATCH TERMINAL RADAR  |  Scenario: " << scenarioName << '\n'
          << "Simulation: " << std::setw(5) << simulationTime << " / "
          << options.durationSeconds << " s  |  Speed: "
          << options.speedMultiplier << "x  |  Update: "
          << options.updateRateHz << " Hz\n"
          << '+' << std::string(static_cast<std::size_t>(radarWidth), '-') << "+\n";

    for (const auto& row : radar) {
        frame << '|' << row << "|\n";
    }

    frame << '+' << std::string(static_cast<std::size_t>(radarWidth), '-') << "+\n"
          << "A/B aircraft  . projected path  X predicted CPA  * overlap\n"
          << "Status: " << conflictStatus(approach)
          << "  |  TCPA: " << approach.timeSeconds << " s"
          << "  |  H-CPA: " << approach.horizontalSeparationMeters << " m"
          << "  |  V-CPA: " << approach.verticalSeparationMeters << " m\n"
          << "Current separation: horizontal " << currentHorizontal
          << " m, vertical " << currentVertical << " m\n"
          << "A" << aircraftA.id() << "  x=" << std::setw(8)
          << aircraftA.position().x << "  y=" << std::setw(8)
          << aircraftA.position().y << "  altitude="
          << aircraftA.position().z << " m\n"
          << "B" << aircraftB.id() << "  x=" << std::setw(8)
          << aircraftB.position().x << "  y=" << std::setw(8)
          << aircraftB.position().y << "  altitude="
          << aircraftB.position().z << " m\n";

    if (redrawInPlace) {
        std::cout << "\x1b[H";
    }
    std::cout << frame.str() << std::flush;
}

} // namespace

void runTerminalSimulation(
    std::string_view scenarioName,
    Aircraft aircraftA,
    Aircraft aircraftB,
    const ConflictDetector& detector,
    TerminalSimulationOptions options) {
    if (options.speedMultiplier <= 0.0 || options.durationSeconds <= 0.0 ||
        options.updateRateHz <= 0.0) {
        throw std::invalid_argument{"Simulation options must be positive"};
    }

    const Bounds bounds =
        calculateBounds(aircraftA, aircraftB, options.durationSeconds);
    const bool redrawInPlace = isatty(STDOUT_FILENO) != 0;
    if (redrawInPlace) {
        std::cout << "\x1b[2J";
    }

    using Clock = std::chrono::steady_clock;
    const auto frameInterval = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>{1.0 / options.updateRateHz});
    auto previousFrame = Clock::now();
    auto nextFrame = previousFrame;
    double simulationTime = 0.0;
    bool firstFrame = true;

    while (true) {
        if (!firstFrame) {
            const auto currentFrame = Clock::now();
            const double realDeltaSeconds =
                std::chrono::duration<double>{currentFrame - previousFrame}.count();
            const double simulationDelta = std::min(
                realDeltaSeconds * options.speedMultiplier,
                options.durationSeconds - simulationTime);

            aircraftA.update(simulationDelta);
            aircraftB.update(simulationDelta);
            simulationTime += simulationDelta;
            previousFrame = currentFrame;
        }

        const ClosestApproach approach = detector.evaluate(aircraftA, aircraftB);
        renderFrame(
            scenarioName,
            aircraftA,
            aircraftB,
            approach,
            bounds,
            options,
            simulationTime,
            redrawInPlace);

        if (simulationTime >= options.durationSeconds) {
            break;
        }

        nextFrame += frameInterval;
        std::this_thread::sleep_until(nextFrame);
        firstFrame = false;
    }

    std::cout << "\nSimulation complete.\n";
}

} // namespace vectorwatch
