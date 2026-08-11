#pragma once

#include "vectorwatch/model/Aircraft.hpp"

#include <optional>

namespace vectorwatch {

struct CollisionConfig {
    double distanceMeters{50.0};
};

struct Collision {
    double timeSeconds{};
    Vector3 position{};
};

class CollisionDetector {
public:
    explicit CollisionDetector(CollisionConfig config = {});

    [[nodiscard]] std::optional<Collision> predict(
        const Aircraft& aircraftA,
        const Aircraft& aircraftB,
        double horizonSeconds) const noexcept;

private:
    CollisionConfig config_;
};

} // namespace vectorwatch
