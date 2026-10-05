#pragma once

#include "vectorwatch/simulation/SimulationSession.hpp"
#include <algorithm>

namespace vectorwatch {

// Fixed display extent, shared by terminal and web radar. Encounter bounds
// take precedence; otherwise include both aircraft's entire nominal run.
inline WorldBounds radarBoundsFor(
    const Aircraft& aircraftA,
    const Aircraft& aircraftB,
    double durationSeconds,
    Optional<WorldBounds> worldBounds = {}) {
    if (worldBounds.hasValue()) return *worldBounds;

    const Vector3 endA =
        aircraftA.position() + (aircraftA.velocity() * durationSeconds);
    const Vector3 endB =
        aircraftB.position() + (aircraftB.velocity() * durationSeconds);
    const double minimumX = std::min(
        {aircraftA.position().x, aircraftB.position().x, endA.x, endB.x});
    const double maximumX = std::max(
        {aircraftA.position().x, aircraftB.position().x, endA.x, endB.x});
    const double minimumY = std::min(
        {aircraftA.position().y, aircraftB.position().y, endA.y, endB.y});
    const double maximumY = std::max(
        {aircraftA.position().y, aircraftB.position().y, endA.y, endB.y});
    const double xPadding = std::max((maximumX - minimumX) * 0.1, 1'000.0);
    const double yPadding = std::max((maximumY - minimumY) * 0.1, 1'000.0);
    return {minimumX - xPadding, maximumX + xPadding,
            minimumY - yPadding, maximumY + yPadding};
}

} // namespace vectorwatch
