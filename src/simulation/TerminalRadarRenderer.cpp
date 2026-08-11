#include "vectorwatch/simulation/TerminalRadarRenderer.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

#include <unistd.h>

namespace vectorwatch {
namespace {

constexpr int radarWidth = 61;
constexpr int radarHeight = 21;

} // namespace

TerminalRadarRenderer::TerminalRadarRenderer(
    const Aircraft& aircraftA,
    const Aircraft& aircraftB,
    double durationSeconds)
    : redrawInPlace_{isatty(STDOUT_FILENO) != 0} {
    const Vector3 endA =
        aircraftA.position() + (aircraftA.velocity() * durationSeconds);
    const Vector3 endB =
        aircraftB.position() + (aircraftB.velocity() * durationSeconds);

    minimumX_ = std::min(
        {aircraftA.position().x, aircraftB.position().x, endA.x, endB.x});
    maximumX_ = std::max(
        {aircraftA.position().x, aircraftB.position().x, endA.x, endB.x});
    minimumY_ = std::min(
        {aircraftA.position().y, aircraftB.position().y, endA.y, endB.y});
    maximumY_ = std::max(
        {aircraftA.position().y, aircraftB.position().y, endA.y, endB.y});

    const double xPadding = std::max((maximumX_ - minimumX_) * 0.1, 1'000.0);
    const double yPadding = std::max((maximumY_ - minimumY_) * 0.1, 1'000.0);
    minimumX_ -= xPadding;
    maximumX_ += xPadding;
    minimumY_ -= yPadding;
    maximumY_ += yPadding;
}

void TerminalRadarRenderer::prepareTerminal() const {
    if (redrawInPlace_) {
        std::cout << "\x1b[2J";
    }
}

TerminalRadarRenderer::GridPoint TerminalRadarRenderer::toGrid(
    const Vector3& position) const {
    const double normalizedX =
        (position.x - minimumX_) / (maximumX_ - minimumX_);
    const double normalizedY =
        (position.y - minimumY_) / (maximumY_ - minimumY_);

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

void TerminalRadarRenderer::drawPath(
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

void TerminalRadarRenderer::drawMarker(
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

std::string_view TerminalRadarRenderer::conflictStatus(
    const ClosestApproach& approach) noexcept {
    if (approach.conflict) {
        return "PREDICTED CONFLICT";
    }
    if (!approach.hasRelativeMotion) {
        return "NO RELATIVE MOTION";
    }
    if (!approach.isWithinLookahead) {
        return "NO FUTURE CONFLICT";
    }
    return "CLEAR";
}

void TerminalRadarRenderer::render(
    std::string_view scenarioName,
    const Aircraft& aircraftA,
    const Aircraft& aircraftB,
    const ClosestApproach& approach,
    const TerminalSimulationOptions& options,
    double simulationTime) const {
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
    drawPath(radar, toGrid(aircraftA.position()), toGrid(projectedA));
    drawPath(radar, toGrid(aircraftB.position()), toGrid(projectedB));

    if (approach.hasRelativeMotion && approach.isWithinLookahead) {
        const Vector3 cpaA =
            aircraftA.position() + (aircraftA.velocity() * approach.timeSeconds);
        const Vector3 cpaB =
            aircraftB.position() + (aircraftB.velocity() * approach.timeSeconds);
        drawMarker(radar, toGrid((cpaA + cpaB) * 0.5), 'X');
    }

    drawMarker(radar, toGrid(aircraftA.position()), 'A');
    drawMarker(radar, toGrid(aircraftB.position()), 'B');

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
          << "Markers: A/B aircraft  . projected path  X predicted CPA  * overlap\n"
          << "Acronyms: CPA = Closest Point of Approach\n"
          << "          TCPA = Time to Closest Point of Approach\n"
          << "          H-CPA = Horizontal Separation at Closest Point of Approach\n"
          << "          V-CPA = Vertical Separation at Closest Point of Approach\n"
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

    if (redrawInPlace_) {
        std::cout << "\x1b[H";
    }
    std::cout << frame.str() << std::flush;
}

void TerminalRadarRenderer::animateCollision(
    std::string_view scenarioName,
    const Aircraft& aircraftA,
    const Aircraft& aircraftB,
    const Vector3& collisionPosition,
    double simulationTime) const {
    constexpr int animationFrameCount = 6;
    constexpr auto animationDelay = std::chrono::milliseconds{120};
    const GridPoint center = toGrid(collisionPosition);

    for (int animationFrame = 0;
         animationFrame < animationFrameCount;
         ++animationFrame) {
        std::vector<std::string> radar(
            radarHeight,
            std::string(static_cast<std::size_t>(radarWidth), ' '));
        drawMarker(radar, toGrid(aircraftA.position()), 'A');
        drawMarker(radar, toGrid(aircraftB.position()), 'B');

        const auto plot = [&](int offsetX, int offsetY, char marker) {
            const int x = center.x + offsetX;
            const int y = center.y + offsetY;
            if (x >= 0 && x < radarWidth && y >= 0 && y < radarHeight) {
                radar[y][x] = marker;
            }
        };
        const auto plotCardinals = [&](int radius, char marker) {
            plot(radius, 0, marker);
            plot(-radius, 0, marker);
            plot(0, radius, marker);
            plot(0, -radius, marker);
        };
        const auto plotDiagonals = [&](int radius, char marker) {
            plot(radius, radius, marker);
            plot(radius, -radius, marker);
            plot(-radius, radius, marker);
            plot(-radius, -radius, marker);
        };

        switch (animationFrame) {
        case 0:
            plot(0, 0, 'X');
            break;
        case 1:
            plot(0, 0, '#');
            plotCardinals(1, '*');
            break;
        case 2:
            plot(0, 0, '@');
            plotCardinals(2, '*');
            plotDiagonals(1, '+');
            break;
        case 3:
            plot(0, 0, '#');
            plotCardinals(1, '*');
            plotDiagonals(2, '*');
            plotCardinals(3, '.');
            break;
        case 4:
            plot(0, 0, '*');
            plotCardinals(2, '+');
            plotDiagonals(3, '.');
            break;
        default:
            plot(0, 0, '.');
            plotCardinals(3, '.');
            break;
        }

        std::ostringstream frame;
        frame << std::fixed << std::setprecision(1)
              << "VECTORWATCH COLLISION  |  Scenario: " << scenarioName << '\n'
              << "Simulation stopped at: " << simulationTime << " s\n"
              << '+' << std::string(static_cast<std::size_t>(radarWidth), '-')
              << "+\n";
        for (const auto& row : radar) {
            frame << '|' << row << "|\n";
        }
        frame << '+' << std::string(static_cast<std::size_t>(radarWidth), '-')
              << "+\n"
              << "COLLISION DETECTED - SIMULATION TERMINATED\n";

        if (redrawInPlace_) {
            std::cout << "\x1b[H";
        }
        std::cout << frame.str() << std::flush;
        if (animationFrame + 1 < animationFrameCount) {
            std::this_thread::sleep_for(animationDelay);
        }
    }

    std::cout << "Collision at x=" << collisionPosition.x
              << " m, y=" << collisionPosition.y
              << " m, altitude=" << collisionPosition.z << " m.\n";
}

void TerminalRadarRenderer::finish() const {
    std::cout << "\nSimulation complete.\n";
}

} // namespace vectorwatch
