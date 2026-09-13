#pragma once

#include "vectorwatch/math/Vector3.hpp"
#include "vectorwatch/prediction/PredictionEngine.hpp"
#include "vectorwatch/simulation/SimulationSession.hpp"
#include "vectorwatch/util/Optional.hpp"

#include <cstddef>
#include <cstdint>

namespace vectorwatch {

enum class SimulationExecutionMode {
    Sequential,
    Threaded,
};

struct TerminalSimulationOptions {
    double speedMultiplier{5.0};
    double durationSeconds{60.0};
    double updateRateHz{20.0};
    Optional<Vector3> waypoint{};
    Vector3 windVelocity{};
    Optional<std::uint32_t> randomSeed{};
    PredictionMode predictionMode{PredictionMode::Deterministic};
    SimulationExecutionMode executionMode{SimulationExecutionMode::Sequential};
    std::size_t predictionSampleCount{10'000};
    std::size_t predictionWorkerCount{0};
    std::uint32_t uncertaintySeed{1};
    UncertaintyProfile uncertaintyProfile{UncertaintyProfile::Medium};
    double predictionRateHz{5.0};
    Optional<WorldBounds> worldBounds{};
};

} // namespace vectorwatch
