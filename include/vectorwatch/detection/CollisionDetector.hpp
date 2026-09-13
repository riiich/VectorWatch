#pragma once

#include "vectorwatch/model/Aircraft.hpp"
#include "vectorwatch/util/Optional.hpp"

namespace vectorwatch {

struct CollisionConfig {
    double distanceMeters{50.0};

    explicit CollisionConfig(double distance = 50.0)
        : distanceMeters(distance) {}
};

struct Collision {
    double timeSeconds{};
    Vector3 position{};

    Collision(double time = 0.0, Vector3 collisionPosition = Vector3())
        : timeSeconds(time), position(collisionPosition) {}
};

class CollisionDetector {
public:
    explicit CollisionDetector(
        CollisionConfig config = CollisionConfig());

    Optional<Collision> predict(
        const Aircraft& aircraftA,
        const Aircraft& aircraftB,
        double horizonSeconds) const noexcept;

private:
    CollisionConfig config_;
};

} // namespace vectorwatch
