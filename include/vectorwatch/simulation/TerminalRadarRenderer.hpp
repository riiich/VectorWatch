#pragma once

#include "vectorwatch/prediction/PredictionEngine.hpp"
#include "vectorwatch/simulation/TerminalSimulationOptions.hpp"

#include <string>
#include <vector>

namespace vectorwatch {

class TerminalRadarRenderer {
public:
    TerminalRadarRenderer(
        const Aircraft& aircraftA,
        const Aircraft& aircraftB,
        const TerminalSimulationOptions& options);

    void prepareTerminal() const;
    void render(
        const Aircraft& aircraftA,
        const Aircraft& aircraftB,
        const PredictionResult& prediction,
        const TerminalSimulationOptions& options) const;
    void animateCollision(
        const Aircraft& aircraftA,
        const Aircraft& aircraftB,
        const Vector3& collisionPosition,
        double simulationTime,
        const TerminalSimulationOptions& options) const;
    void finish(
        const StateSnapshot& snapshot,
        const Optional<PredictionResult>& prediction,
        const SimulationResult& result) const;

private:
    struct GridPoint {
        int x;
        int y;
    };

    GridPoint toGrid(const Vector3& position) const;
    static void drawPath(
        std::vector<std::string>& radar,
        GridPoint start,
        GridPoint end);
    static void drawMarker(
        std::vector<std::string>& radar,
        GridPoint position,
        char marker);
    static void drawExplosionFrame(
        std::vector<std::string>& radar,
        std::vector<std::string>& explosionMask,
        GridPoint center,
        int animationFrame);
    double minimumX_{};
    double maximumX_{};
    double minimumY_{};
    double maximumY_{};
    bool redrawInPlace_{};
    mutable bool printedNonInteractiveFrame_{};
};

} // namespace vectorwatch
