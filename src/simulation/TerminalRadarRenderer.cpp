#include "vectorwatch/simulation/TerminalRadarRenderer.hpp"

#include "vectorwatch/platform/TerminalCapabilities.hpp"
#include "vectorwatch/util/Clamp.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

namespace vectorwatch {
namespace {

constexpr int radarWidth = 61;
constexpr int radarHeight = 21;
const char* const explosionOrange = "\x1b[38;2;255;140;0m";
const char* const resetColor = "\x1b[0m";
constexpr double calmWindEpsilon = 1.0e-6;
constexpr int speedBarWidth = 20;
constexpr double speedBarMaximumMetersPerSecond = 300.0;

double speed(const Vector3& velocity) noexcept {
    return std::sqrt(velocity.lengthSquared());
}

std::string speedBar(double metersPerSecond) {
    const double fraction = clampValue(
        metersPerSecond / speedBarMaximumMetersPerSecond,
        0.0,
        1.0);
    const int filled = static_cast<int>(
        std::lround(fraction * static_cast<double>(speedBarWidth)));
    return "[" + std::string(static_cast<std::size_t>(filled), '#') +
        std::string(
            static_cast<std::size_t>(speedBarWidth - filled),
            '-') +
        "]";
}

const char* windEffect(
    const Aircraft& aircraft,
    const Vector3& windVelocity) noexcept {
    if (std::hypot(windVelocity.x, windVelocity.y) < calmWindEpsilon) {
        return "NO WIND";
    }

    // Aircraft stores ground velocity. Subtracting wind gives air velocity,
    // which tells us whether the current opposes or helps the aircraft.
    const Vector3 airVelocity = aircraft.velocity() - windVelocity;
    const double airspeed = std::hypot(airVelocity.x, airVelocity.y);
    const double windSpeed = std::hypot(windVelocity.x, windVelocity.y);
    if (airspeed < calmWindEpsilon) {
        return "CROSSWIND";
    }
    const double alongTrackEffect =
        (airVelocity.x * windVelocity.x) +
        (airVelocity.y * windVelocity.y);
    const double alignment = alongTrackEffect / (airspeed * windSpeed);
    if (alignment < -0.2) {
        return "HEADWIND";
    }
    if (alignment > 0.2) {
        return "TAILWIND";
    }
    return "CROSSWIND";
}

void drawAirCurrent(
    std::vector<std::string>& radar,
    const Vector3& windVelocity) {
    if (std::hypot(windVelocity.x, windVelocity.y) < calmWindEpsilon) {
        return;
    }

    const double horizontal = std::abs(windVelocity.x);
    const double vertical = std::abs(windVelocity.y);
    std::string label{"AIR CURRENT "};
    if (horizontal > vertical * 2.0) {
        label += windVelocity.x < 0.0 ? "W  <<<<<" : "E  >>>>>";
    } else if (vertical > horizontal * 2.0) {
        label += windVelocity.y < 0.0 ? "S  vvvvv" : "N  ^^^^^";
    } else {
        const char horizontalArrow = windVelocity.x < 0.0 ? '<' : '>';
        const char verticalArrow = windVelocity.y < 0.0 ? 'v' : '^';
        label += windVelocity.y < 0.0 ? "S" : "N";
        label += windVelocity.x < 0.0 ? "W  " : "E  ";
        for (int arrow = 0; arrow < 3; ++arrow) {
            label += horizontalArrow;
            label += verticalArrow;
            if (arrow < 2) {
                label += ' ';
            }
        }
    }
    const std::size_t start =
        (static_cast<std::size_t>(radarWidth) - label.size()) / 2U;
    for (std::size_t index = 0; index < label.size(); ++index) {
        radar[0][start + index] = label[index];
    }
}

void appendAircraftVelocity(
    std::ostringstream& frame,
    char marker,
    const Aircraft& aircraft,
    const Vector3& windVelocity) {
    const Vector3 originalAirVelocity = aircraft.velocity() - windVelocity;
    const double originalAirSpeed = speed(originalAirVelocity);
    const double groundSpeed = speed(aircraft.velocity());
    frame << marker << " original air velocity: ("
          << originalAirVelocity.x << ", "
          << originalAirVelocity.y << ", "
          << originalAirVelocity.z << ") m/s\n"
          << "  original airspeed: " << originalAirSpeed << " m/s  "
          << speedBar(originalAirSpeed) << '\n'
          << marker << " actual ground velocity: ("
          << aircraft.velocity().x << ", "
          << aircraft.velocity().y << ", "
          << aircraft.velocity().z << ") m/s\n"
          << "  actual ground speed: " << groundSpeed << " m/s  "
          << speedBar(groundSpeed) << "  "
          << windEffect(aircraft, windVelocity) << '\n';
}

void appendEnvironmentSummary(
    std::ostringstream& frame,
    const TerminalSimulationOptions& options) {
    if (!options.randomSeed.hasValue()) {
        return;
    }

    frame << "Random seed: " << *options.randomSeed << '\n'
          << "Wind-current velocity: ("
          << options.windVelocity.x << ", "
          << options.windVelocity.y << ", "
          << options.windVelocity.z << ") m/s"
          << "  |  Wind speed: " << speed(options.windVelocity)
          << " m/s\n";
}

} // namespace

TerminalRadarRenderer::TerminalRadarRenderer(
    const Aircraft& aircraftA,
    const Aircraft& aircraftB,
    const TerminalSimulationOptions& options)
    : redrawInPlace_{terminalSupportsInPlaceRendering()} {
    if (options.worldBounds.hasValue()) {
        minimumX_ = options.worldBounds->minimumX;
        maximumX_ = options.worldBounds->maximumX;
        minimumY_ = options.worldBounds->minimumY;
        maximumY_ = options.worldBounds->maximumY;
        return;
    }

    const Vector3 endA =
        aircraftA.position() + (aircraftA.velocity() * options.durationSeconds);
    const Vector3 endB =
        aircraftB.position() + (aircraftB.velocity() * options.durationSeconds);

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
        std::cout << "\x1b[2J\x1b[H";
    }
}

TerminalRadarRenderer::GridPoint TerminalRadarRenderer::toGrid(
    const Vector3& position) const {
    const double normalizedX =
        (position.x - minimumX_) / (maximumX_ - minimumX_);
    const double normalizedY =
        (position.y - minimumY_) / (maximumY_ - minimumY_);

    const int x = clampValue(
        static_cast<int>(std::lround(normalizedX * (radarWidth - 1))),
        0,
        radarWidth - 1);
    const int y = clampValue(
        radarHeight - 1 -
            static_cast<int>(std::lround(normalizedY * (radarHeight - 2))),
        1,
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

void TerminalRadarRenderer::drawExplosionFrame(
    std::vector<std::string>& radar,
    std::vector<std::string>& explosionMask,
    GridPoint center,
    int animationFrame) {
    const auto plot = [&](int offsetX, int offsetY, char marker) {
        const int x = center.x + offsetX;
        const int y = center.y + offsetY;
        if (x >= 0 && x < radarWidth && y >= 0 && y < radarHeight) {
            radar[y][x] = marker;
            explosionMask[y][x] = '1';
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
}

void TerminalRadarRenderer::render(
    const Aircraft& aircraftA,
    const Aircraft& aircraftB,
    const PredictionResult& prediction,
    const TerminalSimulationOptions& options) const {
    // A terminal can normally replace the previous frame. Redirected output
    // and some IDE output panels cannot, so printing every frame there would
    // create thousands of repeated lines.
    if (!redrawInPlace_ && printedNonInteractiveFrame_) {
        return;
    }
    printedNonInteractiveFrame_ = true;

    const ClosestApproach& approach = prediction.nominalApproach;
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
    drawAirCurrent(radar, options.windVelocity);

    if (approach.hasRelativeMotion && approach.isWithinLookahead) {
        const Vector3 cpaA =
            aircraftA.position() + (aircraftA.velocity() * approach.timeSeconds);
        const Vector3 cpaB =
            aircraftB.position() + (aircraftB.velocity() * approach.timeSeconds);
        drawMarker(radar, toGrid((cpaA + cpaB) * 0.5), 'X');
    }

    if (options.waypoint.hasValue()) {
        drawMarker(radar, toGrid(*options.waypoint), 'W');
    }

    const GridPoint currentA = toGrid(aircraftA.position());
    const GridPoint currentB = toGrid(aircraftB.position());
    const bool sharesRadarCell =
        currentA.x == currentB.x && currentA.y == currentB.y;
    if (sharesRadarCell) {
        drawMarker(radar, currentA, 'O');
    } else {
        drawMarker(radar, currentA, 'A');
        drawMarker(radar, currentB, 'B');
    }

    const Vector3 currentSeparation =
        aircraftB.position() - aircraftA.position();
    const double currentHorizontal =
        std::hypot(currentSeparation.x, currentSeparation.y);
    const double currentVertical = std::abs(currentSeparation.z);

    std::ostringstream frame;
    frame << std::fixed << std::setprecision(1)
          << '+' << std::string(static_cast<std::size_t>(radarWidth), '-')
          << "+\n";

    for (const auto& row : radar) {
        frame << '|' << row << "|\n";
    }

    frame << '+' << std::string(static_cast<std::size_t>(radarWidth), '-') << "+\n"
          << "A/B aircraft  W waypoint  X closest approach  . path\n"
          << "Status: " << riskLevelName(prediction.riskLevel) << '\n'
          << "Closest approach in " << approach.timeSeconds << " s"
          << "  |  Horizontal: " << approach.horizontalSeparationMeters << " m"
          << "  |  Vertical: " << approach.verticalSeparationMeters << " m\n";

    if (approach.conflictWindow.exists) {
        frame << "Conflict window: " << approach.conflictWindow.startSeconds
              << " to " << approach.conflictWindow.endSeconds << " s\n";
    } else {
        frame << "Conflict window: none\n";
    }

    frame
          << "Conflict probability: "
          << prediction.conflictProbability * 100.0 << "%"
          << "  |  Collision probability: "
          << prediction.collisionProbability * 100.0 << "%\n"
          << "Samples: " << prediction.sampleCount
          << "  |  Workers: " << prediction.workerCount
          << "  |  Prediction: " << prediction.calculationMilliseconds
          << " ms  |  Sequence: " << prediction.snapshotSequence << "\n"
          << "Current separation: horizontal " << currentHorizontal
          << " m, vertical " << currentVertical << " m\n";

    appendAircraftVelocity(frame, 'A', aircraftA, options.windVelocity);
    appendAircraftVelocity(frame, 'B', aircraftB, options.windVelocity);
    appendEnvironmentSummary(frame, options);

    if (sharesRadarCell) {
        frame << "Radar overlap only: no 3D collision detected; vertical separation "
              << currentVertical << " m\n";
    }

    if (redrawInPlace_) {
        std::cout << "\x1b[2J\x1b[H";
    }
    std::cout << frame.str() << std::flush;
}

void TerminalRadarRenderer::animateCollision(
    const Aircraft& aircraftA,
    const Aircraft& aircraftB,
    const Vector3& collisionPosition,
    double simulationTime,
    const TerminalSimulationOptions& options) const {
    constexpr int animationFrameCount = 6;
    constexpr auto animationDelay = std::chrono::milliseconds{120};
    const GridPoint center = toGrid(collisionPosition);
    const int firstAnimationFrame = redrawInPlace_
        ? 0
        : animationFrameCount - 1;

    for (int animationFrame = firstAnimationFrame;
         animationFrame < animationFrameCount;
         ++animationFrame) {
        std::vector<std::string> radar(
            radarHeight,
            std::string(static_cast<std::size_t>(radarWidth), ' '));
        std::vector<std::string> explosionMask(
            radarHeight,
            std::string(static_cast<std::size_t>(radarWidth), '0'));
        drawAirCurrent(radar, options.windVelocity);
        drawMarker(radar, toGrid(aircraftA.position()), 'A');
        drawMarker(radar, toGrid(aircraftB.position()), 'B');
        drawExplosionFrame(radar, explosionMask, center, animationFrame);

        std::ostringstream frame;
        frame << std::fixed << std::setprecision(1)
              << '+' << std::string(static_cast<std::size_t>(radarWidth), '-')
              << "+\n";
        for (int y = 0; y < radarHeight; ++y) {
            frame << '|';
            for (int x = 0; x < radarWidth; ++x) {
                if (redrawInPlace_ && explosionMask[y][x] == '1') {
                    frame << explosionOrange << radar[y][x] << resetColor;
                } else {
                    frame << radar[y][x];
                }
            }
            frame << "|\n";
        }
        frame << '+' << std::string(static_cast<std::size_t>(radarWidth), '-')
              << "+\n"
              << "Status: CRASHED  |  Collision time: "
              << simulationTime << " s\n";
        appendAircraftVelocity(frame, 'A', aircraftA, options.windVelocity);
        appendAircraftVelocity(frame, 'B', aircraftB, options.windVelocity);
        appendEnvironmentSummary(frame, options);
        frame << "COLLISION DETECTED - SIMULATION TERMINATED\n";

        if (redrawInPlace_) {
            std::cout << "\x1b[2J\x1b[H";
        }
        std::cout << frame.str() << std::flush;
        if (animationFrame + 1 < animationFrameCount) {
            std::this_thread::sleep_for(animationDelay);
        }
    }

}

void TerminalRadarRenderer::finish(
    const StateSnapshot& snapshot,
    const Optional<PredictionResult>& prediction,
    const SimulationResult& result) const {
    const Vector3 separation =
        snapshot.aircraftB.position() - snapshot.aircraftA.position();

    std::cout << std::fixed << std::setprecision(1)
              << "\nFINAL RESULTS\n"
              << "Outcome: ";
    switch (result.reason) {
    case SimulationEndReason::Completed:
        std::cout << "COMPLETED";
        break;
    case SimulationEndReason::Collision:
        std::cout << "COLLISION";
        break;
    case SimulationEndReason::Cancelled:
        std::cout << "CANCELLED";
        break;
    }
    std::cout << "  |  Final simulation time: "
              << result.simulationTimeSeconds << " s\n";

    if (prediction.hasValue()) {
        std::cout << "Final predicted status: "
                  << riskLevelName(prediction->riskLevel)
                  << "  |  Conflict probability: "
                  << prediction->conflictProbability * 100.0 << "%"
                  << "  |  Collision probability: "
                  << prediction->collisionProbability * 100.0 << "%\n";
    }

    std::cout << "Final separation: horizontal "
              << std::hypot(separation.x, separation.y)
              << " m, vertical " << std::abs(separation.z) << " m\n"
              << "A final position: (" << snapshot.aircraftA.position().x
              << ", " << snapshot.aircraftA.position().y
              << ", " << snapshot.aircraftA.position().z << ") m\n"
              << "B final position: (" << snapshot.aircraftB.position().x
              << ", " << snapshot.aircraftB.position().y
              << ", " << snapshot.aircraftB.position().z << ") m\n";

    if (result.collision.hasValue()) {
        std::cout << "Collision position: ("
                  << result.collision->position.x << ", "
                  << result.collision->position.y << ", "
                  << result.collision->position.z << ") m\n";
    }
    if (result.reason == SimulationEndReason::Completed) {
        std::cout << "Simulation complete.\n";
    }
}

} // namespace vectorwatch
