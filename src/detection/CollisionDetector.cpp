#include "vectorwatch/detection/CollisionDetector.hpp"

#include <cmath>
#include <stdexcept>

namespace vectorwatch {
namespace {

constexpr double relativeVelocityEpsilonSquared = 1.0e-12;

} // namespace

CollisionDetector::CollisionDetector(CollisionConfig config) : config_{config} {
    if (!std::isfinite(config_.distanceMeters) || config_.distanceMeters < 0.0) {
        throw std::invalid_argument{
            "Collision distance must be finite and non-negative"};
    }
}

std::optional<Collision> CollisionDetector::predict(
    const Aircraft& aircraftA,
    const Aircraft& aircraftB,
    double horizonSeconds) const noexcept {
    if (!std::isfinite(horizonSeconds) || horizonSeconds < 0.0) {
        return std::nullopt;
    }

    const Vector3 relativePosition = aircraftB.position() - aircraftA.position();
    const Vector3 relativeVelocity = aircraftB.velocity() - aircraftA.velocity();
    const double relativeSpeedSquared = relativeVelocity.lengthSquared();
    const double collisionDistanceSquared =
        config_.distanceMeters * config_.distanceMeters;
    const double currentDistanceSquared = relativePosition.lengthSquared();

    double collisionTimeSeconds = 0.0;
    if (currentDistanceSquared > collisionDistanceSquared) {
        if (relativeSpeedSquared < relativeVelocityEpsilonSquared) {
            return std::nullopt;
        }

        const double linearCoefficient =
            2.0 * relativePosition.dot(relativeVelocity);
        const double constantCoefficient =
            currentDistanceSquared - collisionDistanceSquared;
        const double discriminant =
            (linearCoefficient * linearCoefficient) -
            (4.0 * relativeSpeedSquared * constantCoefficient);
        if (discriminant < 0.0) {
            return std::nullopt;
        }

        collisionTimeSeconds =
            (-linearCoefficient - std::sqrt(discriminant)) /
            (2.0 * relativeSpeedSquared);
        if (collisionTimeSeconds < 0.0 ||
            collisionTimeSeconds > horizonSeconds) {
            return std::nullopt;
        }
    }

    const Vector3 positionA =
        aircraftA.position() + (aircraftA.velocity() * collisionTimeSeconds);
    const Vector3 positionB =
        aircraftB.position() + (aircraftB.velocity() * collisionTimeSeconds);
    return Collision{
        .timeSeconds = collisionTimeSeconds,
        .position = (positionA + positionB) * 0.5,
    };
}

} // namespace vectorwatch
