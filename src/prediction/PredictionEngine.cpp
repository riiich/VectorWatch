#include "vectorwatch/prediction/PredictionEngine.hpp"
#include "vectorwatch/prediction/PredictionWorkerPool.hpp"
#include "vectorwatch/util/Clamp.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>
#include <utility>

namespace vectorwatch {
namespace {

using Clock = std::chrono::steady_clock;
constexpr double pi = 3.14159265358979323846;

void validateFiniteNonNegative(double value, const std::string& name) {
    if (!std::isfinite(value) || value < 0.0) {
        throw std::invalid_argument{
            std::string{name} + " must be finite and non-negative"};
    }
}

void validateConfig(const PredictionEngineConfig& config) {
    if (config.sampleCount == 0) {
        throw std::invalid_argument{"Prediction sample count must be positive"};
    }
    if (config.chunkCount == 0) {
        throw std::invalid_argument{"Prediction chunk count must be positive"};
    }
    validateFiniteNonNegative(
        config.uncertainty.horizontalPositionStdDevMeters,
        "Horizontal position uncertainty");
    validateFiniteNonNegative(
        config.uncertainty.altitudeStdDevMeters,
        "Altitude uncertainty");
    validateFiniteNonNegative(
        config.uncertainty.airspeedStdDevMetersPerSecond,
        "Airspeed uncertainty");
    validateFiniteNonNegative(
        config.uncertainty.headingStdDevDegrees,
        "Heading uncertainty");
    validateFiniteNonNegative(
        config.uncertainty.verticalRateStdDevMetersPerSecond,
        "Vertical-rate uncertainty");
    validateFiniteNonNegative(
        config.uncertainty.windComponentStdDevMetersPerSecond,
        "Wind uncertainty");
    if (!std::isfinite(config.riskThresholds.potentialProbability) ||
        !std::isfinite(config.riskThresholds.highlyLikelyProbability) ||
        config.riskThresholds.potentialProbability < 0.0 ||
        config.riskThresholds.highlyLikelyProbability > 1.0 ||
        config.riskThresholds.potentialProbability >=
            config.riskThresholds.highlyLikelyProbability) {
        throw std::invalid_argument{"Invalid probability risk thresholds"};
    }
}

double horizontalLength(const Vector3& vector) noexcept {
    return std::hypot(vector.x, vector.y);
}

double standardNormal(
    std::mt19937& engine,
    std::normal_distribution<double>& distribution) {
    return clampValue(distribution(engine), -3.0, 3.0);
}

Aircraft perturbAircraft(
    const Aircraft& aircraft,
    const Vector3& nominalWind,
    const Vector3& sampledWind,
    const UncertaintyConfig& uncertainty,
    std::mt19937& engine,
    std::normal_distribution<double>& distribution) {
    const Vector3 nominalAirVelocity = aircraft.velocity() - nominalWind;
    const double nominalAirspeed = horizontalLength(nominalAirVelocity);
    const double nominalHeading =
        std::atan2(nominalAirVelocity.y, nominalAirVelocity.x);

    const Vector3 sampledPosition{
        aircraft.position().x + standardNormal(engine, distribution) *
            uncertainty.horizontalPositionStdDevMeters,
        aircraft.position().y + standardNormal(engine, distribution) *
            uncertainty.horizontalPositionStdDevMeters,
        aircraft.position().z + standardNormal(engine, distribution) *
            uncertainty.altitudeStdDevMeters,
    };
    const double sampledAirspeed = std::max(
        0.0,
        nominalAirspeed + standardNormal(engine, distribution) *
            uncertainty.airspeedStdDevMetersPerSecond);
    const double sampledHeading = nominalHeading +
        standardNormal(engine, distribution) *
            uncertainty.headingStdDevDegrees * pi / 180.0;
    const double sampledVerticalRate = nominalAirVelocity.z +
        standardNormal(engine, distribution) *
            uncertainty.verticalRateStdDevMetersPerSecond;
    const Vector3 sampledAirVelocity{
        sampledAirspeed * std::cos(sampledHeading),
        sampledAirspeed * std::sin(sampledHeading),
        sampledVerticalRate,
    };

    return Aircraft{
        aircraft.id(),
        sampledPosition,
        sampledAirVelocity + sampledWind};
}

struct SampleCounts {
    std::size_t conflict{};
    std::size_t collision{};
};

class DeterministicPredictionEngine final : public PredictionEngine {
public:
    explicit DeterministicPredictionEngine(PredictionEngineConfig config)
        : config_{std::move(config)},
          conflictDetector_{config_.detector},
          collisionDetector_{config_.collision} {}

    Optional<PredictionResult> tryPredict(
        const StateSnapshot& snapshot,
        const Vector3&,
        const std::function<bool()>& isObsolete) override {
        if (isObsolete()) {
            return Optional<PredictionResult>();
        }
        const auto start = Clock::now();
        const ClosestApproach approach = conflictDetector_.evaluate(
            snapshot.aircraftA,
            snapshot.aircraftB);
        const Optional<Collision> collision = collisionDetector_.predict(
            snapshot.aircraftA,
            snapshot.aircraftB,
            config_.detector.lookaheadSeconds);
        const double conflictProbability = approach.conflict ? 1.0 : 0.0;
        const double collisionProbability = collision.hasValue() ? 1.0 : 0.0;
        const double milliseconds = std::chrono::duration<double, std::milli>{
            Clock::now() - start}.count();

        PredictionResult result;
        result.snapshotSequence = snapshot.sequence;
        result.simulationTimeSeconds = snapshot.simulationTimeSeconds;
        result.nominalApproach = approach;
        result.nominalCollision = collision;
        result.sampleCount = 1;
        result.conflictCount = approach.conflict ? 1U : 0U;
        result.collisionCount = collision.hasValue() ? 1U : 0U;
        result.conflictProbability = conflictProbability;
        result.collisionProbability = collisionProbability;
        result.riskLevel = classifyRisk(
            conflictProbability,
            config_.riskThresholds);
        result.workerCount = 1;
        result.calculationMilliseconds = milliseconds;
        if (isObsolete()) {
            return Optional<PredictionResult>();
        }
        return result;
    }

private:
    PredictionEngineConfig config_;
    ConflictDetector conflictDetector_;
    CollisionDetector collisionDetector_;
};

class MonteCarloPredictionEngine final : public PredictionEngine {
public:
    explicit MonteCarloPredictionEngine(PredictionEngineConfig config)
        : config_{std::move(config)},
          conflictDetector_{config_.detector},
          collisionDetector_{config_.collision} {
        if (config_.workerCount > 0) {
            workerPool_ =
                std::make_unique<PredictionWorkerPool>(config_.workerCount);
        }
    }

    Optional<PredictionResult> tryPredict(
        const StateSnapshot& snapshot,
        const Vector3& nominalWindVelocity,
        const std::function<bool()>& isObsolete) override {
        if (isObsolete()) {
            return Optional<PredictionResult>();
        }
        const auto start = Clock::now();
        const ClosestApproach nominalApproach = conflictDetector_.evaluate(
            snapshot.aircraftA,
            snapshot.aircraftB);
        const Optional<Collision> nominalCollision =
            collisionDetector_.predict(
                snapshot.aircraftA,
                snapshot.aircraftB,
                config_.detector.lookaheadSeconds);

        const std::size_t chunkCount = std::min(
            config_.chunkCount,
            config_.sampleCount);
        std::vector<SampleCounts> chunkResults(chunkCount);
        std::vector<std::future<void>> completions;
        completions.reserve(chunkCount);
        for (std::size_t chunk = 0; chunk < chunkCount; ++chunk) {
            const std::size_t begin = config_.sampleCount * chunk / chunkCount;
            const std::size_t end =
                config_.sampleCount * (chunk + 1) / chunkCount;
            const auto evaluate = [&, chunk, begin, end] {
                chunkResults[chunk] = evaluateChunk(
                    snapshot,
                    nominalWindVelocity,
                    chunk,
                    begin,
                    end,
                    isObsolete);
            };
            if (workerPool_) {
                completions.push_back(workerPool_->submit(evaluate));
            } else {
                evaluate();
            }
        }

        for (std::future<void>& completion : completions) {
            completion.get();
        }

        if (isObsolete()) {
            return Optional<PredictionResult>();
        }

        SampleCounts total{};
        for (const SampleCounts& chunk : chunkResults) {
            total.conflict += chunk.conflict;
            total.collision += chunk.collision;
        }

        const double conflictProbability =
            static_cast<double>(total.conflict) /
            static_cast<double>(config_.sampleCount);
        const double collisionProbability =
            static_cast<double>(total.collision) /
            static_cast<double>(config_.sampleCount);
        const double milliseconds = std::chrono::duration<double, std::milli>{
            Clock::now() - start}.count();

        PredictionResult result;
        result.snapshotSequence = snapshot.sequence;
        result.simulationTimeSeconds = snapshot.simulationTimeSeconds;
        result.nominalApproach = nominalApproach;
        result.nominalCollision = nominalCollision;
        result.sampleCount = config_.sampleCount;
        result.conflictCount = total.conflict;
        result.collisionCount = total.collision;
        result.conflictProbability = conflictProbability;
        result.collisionProbability = collisionProbability;
        result.riskLevel = classifyRisk(
            conflictProbability,
            config_.riskThresholds);
        result.workerCount = workerPool_ ? workerPool_->size() : 1;
        result.calculationMilliseconds = milliseconds;
        return result;
    }

private:
    PredictionEngineConfig config_;
    ConflictDetector conflictDetector_;
    CollisionDetector collisionDetector_;
    std::unique_ptr<PredictionWorkerPool> workerPool_{};

    SampleCounts evaluateChunk(
        const StateSnapshot& snapshot,
        const Vector3& nominalWindVelocity,
        std::size_t chunk,
        std::size_t begin,
        std::size_t end,
        const std::function<bool()>& isObsolete) const {
        std::seed_seq seedSequence{
            config_.uncertaintySeed,
            static_cast<std::uint32_t>(config_.uncertaintySeed >> 16U),
            static_cast<std::uint32_t>(chunk),
            0x56454354U};
        std::mt19937 engine{seedSequence};
        std::normal_distribution<double> distribution{0.0, 1.0};
        SampleCounts counts{};

        for (std::size_t sample = begin; sample < end; ++sample) {
            if ((sample - begin) % 64U == 0U && isObsolete()) {
                return counts;
            }
            const Vector3 sampledWind{
                nominalWindVelocity.x + standardNormal(engine, distribution) *
                    config_.uncertainty.windComponentStdDevMetersPerSecond,
                nominalWindVelocity.y + standardNormal(engine, distribution) *
                    config_.uncertainty.windComponentStdDevMetersPerSecond,
                nominalWindVelocity.z,
            };
            const Aircraft aircraftA = perturbAircraft(
                snapshot.aircraftA,
                nominalWindVelocity,
                sampledWind,
                config_.uncertainty,
                engine,
                distribution);
            const Aircraft aircraftB = perturbAircraft(
                snapshot.aircraftB,
                nominalWindVelocity,
                sampledWind,
                config_.uncertainty,
                engine,
                distribution);

            if (conflictDetector_.evaluate(aircraftA, aircraftB).conflict) {
                ++counts.conflict;
            }
            if (collisionDetector_
                    .predict(
                        aircraftA,
                        aircraftB,
                        config_.detector.lookaheadSeconds)
                    .hasValue()) {
                ++counts.collision;
            }
        }
        return counts;
    }
};

} // namespace

PredictionResult PredictionEngine::predict(
    const StateSnapshot& snapshot,
    const Vector3& nominalWindVelocity) {
    Optional<PredictionResult> result = tryPredict(
        snapshot,
        nominalWindVelocity,
        [] { return false; });
    if (!result.hasValue()) {
        throw std::runtime_error{"Prediction was unexpectedly cancelled"};
    }
    return std::move(*result);
}

UncertaintyConfig uncertaintyConfigFor(UncertaintyProfile profile) noexcept {
    switch (profile) {
    case UncertaintyProfile::Low:
        return UncertaintyConfig(10.0, 5.0, 1.0, 0.5, 0.25, 0.75);
    case UncertaintyProfile::Medium:
        return UncertaintyConfig();
    case UncertaintyProfile::High:
        return UncertaintyConfig(50.0, 20.0, 4.0, 2.0, 1.0, 3.0);
    }
    return UncertaintyConfig();
}

RiskLevel classifyRisk(
    double conflictProbability,
    RiskThresholds thresholds) {
    if (!std::isfinite(conflictProbability) || conflictProbability < 0.0 ||
        conflictProbability > 1.0) {
        throw std::invalid_argument{"Conflict probability must be from zero to one"};
    }
    if (!std::isfinite(thresholds.potentialProbability) ||
        !std::isfinite(thresholds.highlyLikelyProbability) ||
        thresholds.potentialProbability < 0.0 ||
        thresholds.highlyLikelyProbability > 1.0 ||
        thresholds.potentialProbability >= thresholds.highlyLikelyProbability) {
        throw std::invalid_argument{"Invalid probability risk thresholds"};
    }
    if (conflictProbability >= thresholds.highlyLikelyProbability) {
        return RiskLevel::HighlyLikely;
    }
    if (conflictProbability >= thresholds.potentialProbability) {
        return RiskLevel::Potential;
    }
    return RiskLevel::Low;
}

const char* riskLevelName(RiskLevel level) noexcept {
    switch (level) {
    case RiskLevel::Low:
        return "LOW RISK";
    case RiskLevel::Potential:
        return "POTENTIAL CONFLICT";
    case RiskLevel::HighlyLikely:
        return "HIGHLY LIKELY CONFLICT";
    }
    return "UNKNOWN";
}

std::unique_ptr<PredictionEngine> makePredictionEngine(
    PredictionEngineConfig config) {
    validateConfig(config);
    if (config.mode == PredictionMode::Probabilistic) {
        return std::make_unique<MonteCarloPredictionEngine>(std::move(config));
    }
    return std::make_unique<DeterministicPredictionEngine>(std::move(config));
}

} // namespace vectorwatch
