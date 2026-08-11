#pragma once

#include "vectorwatch/detection/ConflictDetector.hpp"
#include "vectorwatch/simulation/TerminalSimulationOptions.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace vectorwatch {

class TerminalRadarRenderer {
public:
    TerminalRadarRenderer(
        const Aircraft& aircraftA,
        const Aircraft& aircraftB,
        double durationSeconds);

    void prepareTerminal() const;
    void render(
        std::string_view scenarioName,
        const Aircraft& aircraftA,
        const Aircraft& aircraftB,
        const ClosestApproach& approach,
        const TerminalSimulationOptions& options,
        double simulationTime) const;
    void animateCollision(
        std::string_view scenarioName,
        const Aircraft& aircraftA,
        const Aircraft& aircraftB,
        const Vector3& collisionPosition,
        double simulationTime) const;
    void finish() const;

private:
    struct GridPoint {
        int x;
        int y;
    };

    [[nodiscard]] GridPoint toGrid(const Vector3& position) const;
    static void drawPath(
        std::vector<std::string>& radar,
        GridPoint start,
        GridPoint end);
    static void drawMarker(
        std::vector<std::string>& radar,
        GridPoint position,
        char marker);
    [[nodiscard]] static std::string_view conflictStatus(
        const ClosestApproach& approach) noexcept;

    double minimumX_{};
    double maximumX_{};
    double minimumY_{};
    double maximumY_{};
    bool redrawInPlace_{};
};

} // namespace vectorwatch
