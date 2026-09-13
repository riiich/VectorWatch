#include "vectorwatch/detection/ConflictDetector.hpp"
#include "vectorwatch/util/Clamp.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vectorwatch {
namespace {

constexpr double relativeVelocityEpsilon = 1.0e-6;
constexpr double relativeVelocityEpsilonSquared =
    relativeVelocityEpsilon * relativeVelocityEpsilon;

double horizontalLength(const Vector3& vector) noexcept {
    return std::hypot(vector.x, vector.y);
}

TimeWindow horizontalViolationInterval(
    const Vector3& relativePosition,
    const Vector3& relativeVelocity,
    double thresholdMeters,
    double lookaheadSeconds) noexcept {
    if (thresholdMeters <= 0.0) {
        return {};
    }

    const double speedSquared =
        (relativeVelocity.x * relativeVelocity.x) +
        (relativeVelocity.y * relativeVelocity.y);
    const double thresholdSquared = thresholdMeters * thresholdMeters;
    const double currentDistanceSquared =
        (relativePosition.x * relativePosition.x) +
        (relativePosition.y * relativePosition.y);

    if (speedSquared < relativeVelocityEpsilonSquared) {
        return TimeWindow(
            0.0,
            lookaheadSeconds,
            currentDistanceSquared < thresholdSquared);
    }

    const double linearCoefficient =
        2.0 * ((relativePosition.x * relativeVelocity.x) +
               (relativePosition.y * relativeVelocity.y));
    const double constantCoefficient =
        currentDistanceSquared - thresholdSquared;
    const double discriminant =
        (linearCoefficient * linearCoefficient) -
        (4.0 * speedSquared * constantCoefficient);

    // A tangent contact reaches the threshold but never crosses it.
    if (discriminant <= 0.0) {
        return {};
    }

    const double root = std::sqrt(discriminant);
    const double denominator = 2.0 * speedSquared;
    const double firstRoot = (-linearCoefficient - root) / denominator;
    const double secondRoot = (-linearCoefficient + root) / denominator;
    const double startSeconds = std::max(0.0, firstRoot);
    const double endSeconds = std::min(lookaheadSeconds, secondRoot);

    return TimeWindow(startSeconds, endSeconds, startSeconds <= endSeconds);
}

TimeWindow verticalViolationInterval(
    const Vector3& relativePosition,
    const Vector3& relativeVelocity,
    double thresholdMeters,
    double lookaheadSeconds) noexcept {
    if (thresholdMeters <= 0.0) {
        return {};
    }

    if (std::abs(relativeVelocity.z) < relativeVelocityEpsilon) {
        return TimeWindow(
            0.0,
            lookaheadSeconds,
            std::abs(relativePosition.z) < thresholdMeters);
    }

    const double firstRoot =
        (-thresholdMeters - relativePosition.z) / relativeVelocity.z;
    const double secondRoot =
        (thresholdMeters - relativePosition.z) / relativeVelocity.z;
    const double entrySeconds = std::min(firstRoot, secondRoot);
    const double exitSeconds = std::max(firstRoot, secondRoot);
    const double startSeconds = std::max(0.0, entrySeconds);
    const double endSeconds = std::min(lookaheadSeconds, exitSeconds);

    return TimeWindow(startSeconds, endSeconds, startSeconds <= endSeconds);
}

bool violatesThresholdsAt(
    const Vector3& relativePosition,
    const Vector3& relativeVelocity,
    const ConflictThresholds& thresholds,
    double timeSeconds) noexcept {
    const Vector3 separation =
        relativePosition + (relativeVelocity * timeSeconds);
    return horizontalLength(separation) < thresholds.horizontalMeters &&
           std::abs(separation.z) < thresholds.verticalMeters;
}

TimeWindow overlappingViolationWindow(
    const TimeWindow& horizontalInterval,
    const TimeWindow& verticalInterval,
    const Vector3& relativePosition,
    const Vector3& relativeVelocity,
    const ConflictThresholds& thresholds) noexcept {
    if (!horizontalInterval.exists || !verticalInterval.exists) {
        return {};
    }

    const double overlapStart = std::max(
        horizontalInterval.startSeconds,
        verticalInterval.startSeconds);
    const double overlapEnd = std::min(
        horizontalInterval.endSeconds,
        verticalInterval.endSeconds);

    if (overlapStart < overlapEnd) {
        return TimeWindow(overlapStart, overlapEnd, true);
    }

    const bool overlapExists = overlapStart == overlapEnd &&
        violatesThresholdsAt(
            relativePosition,
            relativeVelocity,
            thresholds,
            overlapStart);
    return TimeWindow(overlapStart, overlapEnd, overlapExists);
}

} // namespace

ConflictDetector::ConflictDetector(DetectorConfig config) : config_{config} {
    if (!std::isfinite(config_.lookaheadSeconds) ||
        config_.lookaheadSeconds < 0.0) {
        throw std::invalid_argument{"Lookahead time must be finite and non-negative"};
    }
    if (!std::isfinite(config_.thresholds.horizontalMeters) ||
        !std::isfinite(config_.thresholds.verticalMeters) ||
        config_.thresholds.horizontalMeters < 0.0 ||
        config_.thresholds.verticalMeters < 0.0) {
        throw std::invalid_argument{
            "Separation thresholds must be finite and non-negative"};
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
        timeSeconds = clampValue(
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
    const double horizontalRelativeSpeedSquared =
        (relativeVelocity.x * relativeVelocity.x) +
        (relativeVelocity.y * relativeVelocity.y);
    const double minimumHorizontalTime =
        horizontalRelativeSpeedSquared < relativeVelocityEpsilonSquared
        ? 0.0
        : clampValue(
              -((relativePosition.x * relativeVelocity.x) +
                (relativePosition.y * relativeVelocity.y)) /
                  horizontalRelativeSpeedSquared,
              0.0,
              config_.lookaheadSeconds);
    const Vector3 horizontalSeparationAtMinimum =
        relativePosition + (relativeVelocity * minimumHorizontalTime);
    const double minimumHorizontalSeparation =
        horizontalLength(horizontalSeparationAtMinimum);

    const double minimumVerticalTime =
        std::abs(relativeVelocity.z) < relativeVelocityEpsilon
        ? 0.0
        : clampValue(
              -relativePosition.z / relativeVelocity.z,
              0.0,
              config_.lookaheadSeconds);
    const double minimumVerticalSeparation = std::abs(
        relativePosition.z + (relativeVelocity.z * minimumVerticalTime));

    const TimeWindow horizontalInterval = horizontalViolationInterval(
        relativePosition,
        relativeVelocity,
        config_.thresholds.horizontalMeters,
        config_.lookaheadSeconds);
    const TimeWindow verticalInterval = verticalViolationInterval(
        relativePosition,
        relativeVelocity,
        config_.thresholds.verticalMeters,
        config_.lookaheadSeconds);

    const TimeWindow conflictWindow = overlappingViolationWindow(
        horizontalInterval,
        verticalInterval,
        relativePosition,
        relativeVelocity,
        config_.thresholds);

    ClosestApproach result;
    result.currentDistanceMeters =
        std::sqrt(relativePosition.lengthSquared());
    result.timeSeconds = timeSeconds;
    result.horizontalSeparationMeters = horizontalSeparation;
    result.verticalSeparationMeters = verticalSeparation;
    result.minimumHorizontalSeparationMeters = minimumHorizontalSeparation;
    result.minimumHorizontalTimeSeconds = minimumHorizontalTime;
    result.minimumVerticalSeparationMeters = minimumVerticalSeparation;
    result.minimumVerticalTimeSeconds = minimumVerticalTime;
    result.horizontalViolationWindow = horizontalInterval;
    result.verticalViolationWindow = verticalInterval;
    result.conflictWindow = conflictWindow;
    result.hasRelativeMotion = hasRelativeMotion;
    result.isWithinLookahead = isWithinLookahead;
    result.conflict = conflictWindow.exists;
    return result;
}

} // namespace vectorwatch
