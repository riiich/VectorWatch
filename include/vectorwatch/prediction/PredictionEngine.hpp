#pragma once

#include "vectorwatch/detection/CollisionDetector.hpp"
#include "vectorwatch/detection/ConflictDetector.hpp"
#include "vectorwatch/simulation/SimulationSession.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>

namespace vectorwatch {

enum class PredictionMode {
    Deterministic,
    Probabilistic,
};

enum class RiskLevel {
    Low,
    Potential,
    HighlyLikely,
};

enum class UncertaintyProfile {
    Low,
    Medium,
    High,
};

struct RiskThresholds {
    double potentialProbability{0.05};
    double highlyLikelyProbability{0.70};

    RiskThresholds(double potential = 0.05, double highlyLikely = 0.70)
        : potentialProbability(potential),
          highlyLikelyProbability(highlyLikely) {}
};

struct UncertaintyConfig {
    double horizontalPositionStdDevMeters{25.0};
    double altitudeStdDevMeters{10.0};
    double airspeedStdDevMetersPerSecond{2.0};
    double headingStdDevDegrees{1.0};
    double verticalRateStdDevMetersPerSecond{0.5};
    double windComponentStdDevMetersPerSecond{1.5};

    UncertaintyConfig(
        double horizontalPosition = 25.0,
        double altitude = 10.0,
        double airspeed = 2.0,
        double heading = 1.0,
        double verticalRate = 0.5,
        double windComponent = 1.5)
        : horizontalPositionStdDevMeters(horizontalPosition),
          altitudeStdDevMeters(altitude),
          airspeedStdDevMetersPerSecond(airspeed),
          headingStdDevDegrees(heading),
          verticalRateStdDevMetersPerSecond(verticalRate),
          windComponentStdDevMetersPerSecond(windComponent) {}
};

struct PredictionEngineConfig {
    PredictionMode mode{PredictionMode::Deterministic};
    std::size_t sampleCount{10'000};
    std::uint32_t uncertaintySeed{1};
    std::size_t chunkCount{256};
    std::size_t workerCount{};
    UncertaintyConfig uncertainty{};
    RiskThresholds riskThresholds{};
    DetectorConfig detector{};
    CollisionConfig collision{};
};

struct PredictionResult {
    std::uint64_t snapshotSequence{};
    double simulationTimeSeconds{};
    ClosestApproach nominalApproach{};
    Optional<Collision> nominalCollision{};
    std::size_t sampleCount{};
    std::size_t conflictCount{};
    std::size_t collisionCount{};
    double conflictProbability{};
    double collisionProbability{};
    RiskLevel riskLevel{RiskLevel::Low};
    std::size_t workerCount{1};
    double calculationMilliseconds{};
};

class PredictionEngine {
public:
    virtual ~PredictionEngine() = default;

    PredictionResult predict(
        const StateSnapshot& snapshot,
        const Vector3& nominalWindVelocity);

    virtual Optional<PredictionResult> tryPredict(
        const StateSnapshot& snapshot,
        const Vector3& nominalWindVelocity,
        const std::function<bool()>& isObsolete) = 0;
};

UncertaintyConfig uncertaintyConfigFor(
    UncertaintyProfile profile) noexcept;

RiskLevel classifyRisk(
    double conflictProbability,
    RiskThresholds thresholds = {});

const char* riskLevelName(RiskLevel level) noexcept;

std::unique_ptr<PredictionEngine> makePredictionEngine(
    PredictionEngineConfig config);

} // namespace vectorwatch
