#pragma once

#include "vectorwatch/model/Aircraft.hpp"

namespace vectorwatch {

struct ConflictThresholds {
    double horizontalMeters{1'000.0};
    double verticalMeters{150.0};

    ConflictThresholds(
        double horizontal = 1'000.0,
        double vertical = 150.0)
        : horizontalMeters(horizontal), verticalMeters(vertical) {}
};

struct DetectorConfig {
    double lookaheadSeconds{120.0};
    ConflictThresholds thresholds{};

    DetectorConfig(
        double lookahead = 120.0,
        ConflictThresholds separationThresholds = ConflictThresholds())
        : lookaheadSeconds(lookahead), thresholds(separationThresholds) {}
};

struct TimeWindow {
    double startSeconds{};
    double endSeconds{};
    bool exists{};

    TimeWindow(double start = 0.0, double end = 0.0, bool present = false)
        : startSeconds(start), endSeconds(end), exists(present) {}
};

struct ClosestApproach {
    double currentDistanceMeters{};
    double timeSeconds{};
    double horizontalSeparationMeters{};
    double verticalSeparationMeters{};
    double minimumHorizontalSeparationMeters{};
    double minimumHorizontalTimeSeconds{};
    double minimumVerticalSeparationMeters{};
    double minimumVerticalTimeSeconds{};
    TimeWindow horizontalViolationWindow{};
    TimeWindow verticalViolationWindow{};
    TimeWindow conflictWindow{};
    bool hasRelativeMotion{};
    bool isWithinLookahead{};
    bool conflict{};
};

class ConflictDetector {
public:
    explicit ConflictDetector(DetectorConfig config = DetectorConfig());

    ClosestApproach evaluate(
        const Aircraft& aircraftA,
        const Aircraft& aircraftB) const noexcept;

private:
    DetectorConfig config_;
};

} // namespace vectorwatch
