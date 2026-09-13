#pragma once

#include "vectorwatch/detection/CollisionDetector.hpp"
#include "vectorwatch/model/Aircraft.hpp"
#include "vectorwatch/util/Optional.hpp"

#include <cstdint>
#include <utility>

namespace vectorwatch {

enum class SimulationEndReason {
    Completed,
    Collision,
    Cancelled,
};

struct WorldBounds {
    double minimumX{};
    double maximumX{};
    double minimumY{};
    double maximumY{};

    WorldBounds(
        double minX = 0.0,
        double maxX = 0.0,
        double minY = 0.0,
        double maxY = 0.0)
        : minimumX(minX), maximumX(maxX), minimumY(minY), maximumY(maxY) {}

    bool contains(const Vector3& position) const noexcept;
};

struct StateSnapshot {
    std::uint64_t sequence{};
    double simulationTimeSeconds{};
    Aircraft aircraftA;
    Aircraft aircraftB;

    StateSnapshot(
        std::uint64_t snapshotSequence,
        double time,
        Aircraft firstAircraft,
        Aircraft secondAircraft)
        : sequence(snapshotSequence),
          simulationTimeSeconds(time),
          aircraftA(firstAircraft),
          aircraftB(secondAircraft) {}
};

struct SimulationResult {
    SimulationEndReason reason{SimulationEndReason::Completed};
    double simulationTimeSeconds{};
    Optional<Collision> collision{};

    SimulationResult(
        SimulationEndReason endReason = SimulationEndReason::Completed,
        double time = 0.0,
        Optional<Collision> collisionResult = Optional<Collision>())
        : reason(endReason),
          simulationTimeSeconds(time),
          collision(std::move(collisionResult)) {}
};

class SimulationSession {
public:
    SimulationSession(
        Aircraft aircraftA,
        Aircraft aircraftB,
        double durationSeconds,
        Optional<WorldBounds> worldBounds = Optional<WorldBounds>(),
        CollisionDetector collisionDetector = CollisionDetector{});

    const StateSnapshot& snapshot() const noexcept;
    const Optional<SimulationResult>& result() const noexcept;

    void advance(double deltaTimeSeconds);
    void cancel() noexcept;

private:
    double durationSeconds_{};
    Optional<WorldBounds> worldBounds_{};
    CollisionDetector collisionDetector_{};
    StateSnapshot snapshot_;
    Optional<SimulationResult> result_{};

    void finish(
        SimulationEndReason reason,
        Optional<Collision> collision = Optional<Collision>()) noexcept;
};

} // namespace vectorwatch
