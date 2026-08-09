#include "vectorwatch/ConflictDetector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vectorwatch {
namespace {

constexpr double relativeVelocityEpsilonSquared = 1.0e-12;

[[nodiscard]] double horizontalLength(const Vector3& vector) noexcept {
    return std::hypot(vector.x, vector.y);
}

} // namespace

ConflictDetector::ConflictDetector(DetectorConfig config) : config_{config} {
    if (config_.lookaheadSeconds < 0.0) {
        throw std::invalid_argument{"Lookahead time cannot be negative"};
    }
    if (config_.thresholds.horizontalMeters < 0.0 ||
        config_.thresholds.verticalMeters < 0.0) {
        throw std::invalid_argument{"Separation thresholds cannot be negative"};
    }
}

ClosestApproach ConflictDetector::evaluate(
    const Aircraft& aircraftA,
    const Aircraft& aircraftB) const noexcept {
    const Vector3 relativePosition = aircraftB.position() - aircraftA.position();
    const Vector3 relativeVelocity = aircraftB.velocity() - aircraftA.velocity();
    const double relativeSpeedSquared = relativeVelocity.lengthSquared();

    const bool hasRelativeMotion =
        relativeSpeedSquared >= relativeVelocityEpsilonSquared;

    double timeSeconds = 0.0;
    bool isWithinLookahead = true;
    if (hasRelativeMotion) {
        const double unconstrainedTime =
            -relativePosition.dot(relativeVelocity) / relativeSpeedSquared;
        isWithinLookahead =
            unconstrainedTime >= 0.0 &&
            unconstrainedTime <= config_.lookaheadSeconds;
        timeSeconds = std::clamp(
            unconstrainedTime,
            0.0,
            config_.lookaheadSeconds);
    }

    const Vector3 separationAtClosestApproach =
        relativePosition + (relativeVelocity * timeSeconds);
    const double horizontalSeparation =
        horizontalLength(separationAtClosestApproach);
    const double verticalSeparation =
        std::abs(separationAtClosestApproach.z);

    return {
        .currentDistanceMeters = std::sqrt(relativePosition.lengthSquared()),
        .timeSeconds = timeSeconds,
        .horizontalSeparationMeters = horizontalSeparation,
        .verticalSeparationMeters = verticalSeparation,
        .hasRelativeMotion = hasRelativeMotion,
        .isWithinLookahead = isWithinLookahead,
        .conflict =
            isWithinLookahead &&
            horizontalSeparation < config_.thresholds.horizontalMeters &&
            verticalSeparation < config_.thresholds.verticalMeters,
    };
}

} // namespace vectorwatch
