#pragma once

#include "vectorwatch/model/Aircraft.hpp"

namespace vectorwatch {

struct ConflictThresholds {
    double horizontalMeters{1'000.0};
    double verticalMeters{150.0};
};

struct DetectorConfig {
    double lookaheadSeconds{120.0};
    ConflictThresholds thresholds{};
};

struct ClosestApproach {
    double currentDistanceMeters{};
    double timeSeconds{};
    double horizontalSeparationMeters{};
    double verticalSeparationMeters{};
    bool hasRelativeMotion{};
    bool isWithinLookahead{};
    bool conflict{};
};

class ConflictDetector {
public:
    explicit ConflictDetector(DetectorConfig config = {});

    [[nodiscard]] ClosestApproach evaluate(
        const Aircraft& aircraftA,
        const Aircraft& aircraftB) const noexcept;

private:
    DetectorConfig config_;
};

} // namespace vectorwatch
