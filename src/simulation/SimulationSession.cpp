#include "vectorwatch/simulation/SimulationSession.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace vectorwatch {

bool WorldBounds::contains(const Vector3& position) const noexcept {
    return position.x >= minimumX && position.x <= maximumX &&
        position.y >= minimumY && position.y <= maximumY;
}

SimulationSession::SimulationSession(
    Aircraft aircraftA,
    Aircraft aircraftB,
    double durationSeconds,
    Optional<WorldBounds> worldBounds,
    CollisionDetector collisionDetector)
    : durationSeconds_{durationSeconds},
      worldBounds_{std::move(worldBounds)},
      collisionDetector_{std::move(collisionDetector)},
      snapshot_{0, 0.0, std::move(aircraftA), std::move(aircraftB)} {
    if (!std::isfinite(durationSeconds_) || durationSeconds_ <= 0.0) {
        throw std::invalid_argument{
            "Simulation duration must be finite and positive"};
    }
    if (worldBounds_.hasValue() &&
        (!std::isfinite(worldBounds_->minimumX) ||
         !std::isfinite(worldBounds_->maximumX) ||
         !std::isfinite(worldBounds_->minimumY) ||
         !std::isfinite(worldBounds_->maximumY) ||
         worldBounds_->minimumX >= worldBounds_->maximumX ||
         worldBounds_->minimumY >= worldBounds_->maximumY)) {
        throw std::invalid_argument{"World bounds must be finite and ordered"};
    }

    if (const auto collision = collisionDetector_.predict(
            snapshot_.aircraftA,
            snapshot_.aircraftB,
            0.0)) {
        finish(SimulationEndReason::Collision, collision);
    }
}

const StateSnapshot& SimulationSession::snapshot() const noexcept {
    return snapshot_;
}

const Optional<SimulationResult>& SimulationSession::result() const noexcept {
    return result_;
}

void SimulationSession::advance(double deltaTimeSeconds) {
    if (!std::isfinite(deltaTimeSeconds) || deltaTimeSeconds < 0.0) {
        throw std::invalid_argument{
            "Simulation time step must be finite and non-negative"};
    }
    if (result_.hasValue()) {
        return;
    }

    const double remainingSeconds =
        durationSeconds_ - snapshot_.simulationTimeSeconds;
    const double appliedDelta = std::min(deltaTimeSeconds, remainingSeconds);
    if (const auto collision = collisionDetector_.predict(
            snapshot_.aircraftA,
            snapshot_.aircraftB,
            appliedDelta)) {
        snapshot_.aircraftA.update(collision->timeSeconds);
        snapshot_.aircraftB.update(collision->timeSeconds);
        snapshot_.simulationTimeSeconds += collision->timeSeconds;
        ++snapshot_.sequence;
        finish(SimulationEndReason::Collision, collision);
        return;
    }

    snapshot_.aircraftA.update(appliedDelta);
    snapshot_.aircraftB.update(appliedDelta);
    snapshot_.simulationTimeSeconds += appliedDelta;
    ++snapshot_.sequence;

    const bool bothOutside = worldBounds_.hasValue() &&
        !worldBounds_->contains(snapshot_.aircraftA.position()) &&
        !worldBounds_->contains(snapshot_.aircraftB.position());
    if (bothOutside || snapshot_.simulationTimeSeconds >= durationSeconds_) {
        finish(SimulationEndReason::Completed);
    }
}

void SimulationSession::cancel() noexcept {
    if (!result_.hasValue()) {
        finish(SimulationEndReason::Cancelled);
    }
}

void SimulationSession::finish(
    SimulationEndReason reason,
    Optional<Collision> collision) noexcept {
    result_ = SimulationResult(
        reason,
        snapshot_.simulationTimeSeconds,
        std::move(collision));
}

} // namespace vectorwatch
