#include "vectorwatch/detection/ConflictDetector.hpp"

#include "CollisionDetectorTests.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string_view>

namespace {

using vectorwatch::Aircraft;
using vectorwatch::ClosestApproach;
using vectorwatch::ConflictDetector;
using vectorwatch::DetectorConfig;
using vectorwatch::Vector3;

int failureCount = 0;

void expect(bool condition, std::string_view testName) {
    if (!condition) {
        std::cerr << "FAIL: " << testName << '\n';
        ++failureCount;
    }
}

void expectNear(
    double actual,
    double expected,
    double tolerance,
    std::string_view testName) {
    expect(std::abs(actual - expected) <= tolerance, testName);
}

[[nodiscard]] ClosestApproach evaluate(
    Vector3 positionA,
    Vector3 velocityA,
    Vector3 positionB,
    Vector3 velocityB,
    DetectorConfig config = {}) {
    return ConflictDetector{config}.evaluate(
        Aircraft{1, positionA, velocityA},
        Aircraft{2, positionB, velocityB});
}

void testHeadOnConflict() {
    const auto result = evaluate(
        {-10'000.0, 0.0, 10'000.0},
        {200.0, 0.0, 0.0},
        {10'000.0, 0.0, 10'000.0},
        {-200.0, 0.0, 0.0});

    expect(result.conflict, "head-on aircraft conflict");
    expectNear(result.timeSeconds, 50.0, 1.0e-9, "head-on TCPA");
    expectNear(
        result.horizontalSeparationMeters,
        0.0,
        1.0e-9,
        "head-on horizontal CPA");
}

void testNearMissConflict() {
    const auto result = evaluate(
        {-10'000.0, 0.0, 10'000.0},
        {200.0, 0.0, 0.0},
        {10'000.0, 600.0, 10'080.0},
        {-200.0, 0.0, 0.0});

    expect(result.conflict, "near miss inside both thresholds");
    expectNear(
        result.horizontalSeparationMeters,
        600.0,
        1.0e-9,
        "near-miss horizontal CPA");
    expectNear(
        result.verticalSeparationMeters,
        80.0,
        1.0e-9,
        "near-miss vertical CPA");
}

void testStaggeredCrossingIsSafe() {
    const auto result = evaluate(
        {-10'000.0, 0.0, 10'000.0},
        {200.0, 0.0, 0.0},
        {0.0, -16'000.0, 10'000.0},
        {0.0, 200.0, 0.0});

    expect(!result.conflict, "staggered crossing remains safe");
}

void testOverlappingDangerWindows() {
    const auto result = evaluate(
        {0.0, 0.0, 10'000.0},
        {0.0, 0.0, 0.0},
        {2'000.0, 0.0, 10'700.0},
        {-200.0, 0.0, -50.0});

    expect(
        result.verticalSeparationMeters > 150.0,
        "3D CPA alone is outside vertical threshold");
    expect(
        result.conflict,
        "overlapping horizontal and vertical danger windows conflict");
}

void testSeparatedDangerWindows() {
    const auto result = evaluate(
        {0.0, 0.0, 10'000.0},
        {0.0, 0.0, 0.0},
        {2'000.0, 0.0, 11'250.0},
        {-200.0, 0.0, -50.0});

    expect(
        !result.conflict,
        "non-overlapping horizontal and vertical danger windows are safe");
}

void testVerticalSeparationIsSafe() {
    const auto result = evaluate(
        {-10'000.0, 0.0, 9'000.0},
        {200.0, 0.0, 0.0},
        {0.0, -10'000.0, 12'000.0},
        {0.0, 200.0, 0.0});

    expect(!result.conflict, "crossing at different altitudes is safe");
}

void testVerticalConvergence() {
    const auto result = evaluate(
        {0.0, 0.0, 10'000.0},
        {200.0, 0.0, 10.0},
        {500.0, 0.0, 10'700.0},
        {200.0, 0.0, -40.0});

    expect(result.conflict, "vertical convergence within horizontal range");
}

void testDivergingAircraft() {
    const auto result = evaluate(
        {-1'000.0, 0.0, 10'000.0},
        {-200.0, 0.0, 0.0},
        {1'000.0, 0.0, 10'000.0},
        {200.0, 0.0, 0.0});

    expect(!result.conflict, "diverging aircraft have no future conflict");
    expect(!result.isWithinLookahead, "diverging CPA is in the past");
}

void testConflictOutsideLookahead() {
    const auto result = evaluate(
        {-30'000.0, 0.0, 10'000.0},
        {100.0, 0.0, 0.0},
        {30'000.0, 0.0, 10'000.0},
        {-100.0, 0.0, 0.0});

    expect(!result.conflict, "conflict beyond lookahead is ignored");
    expect(!result.isWithinLookahead, "CPA is beyond lookahead");
    expectNear(result.timeSeconds, 120.0, 1.0e-9, "TCPA clamps to lookahead");
}

void testZeroRelativeVelocity() {
    const Vector3 sharedVelocity{200.0, 0.0, 0.0};
    const auto safeResult = evaluate(
        {0.0, 0.0, 10'000.0},
        sharedVelocity,
        {0.0, 5'000.0, 10'000.0},
        sharedVelocity);
    const auto conflictResult = evaluate(
        {0.0, 0.0, 10'000.0},
        sharedVelocity,
        {500.0, 0.0, 10'100.0},
        sharedVelocity);

    expect(!safeResult.hasRelativeMotion, "same velocity has no relative motion");
    expect(!safeResult.conflict, "same velocity with safe spacing");
    expect(conflictResult.conflict, "same velocity with unsafe spacing");
}

void testStrictThresholdBoundary() {
    const auto result = evaluate(
        {-2'000.0, 0.0, 10'000.0},
        {200.0, 0.0, 0.0},
        {0.0, 1'000.0, 10'000.0},
        {0.0, 0.0, 0.0});

    expect(!result.conflict, "exact horizontal threshold is not a conflict");
    expectNear(
        result.horizontalSeparationMeters,
        1'000.0,
        1.0e-9,
        "exact horizontal threshold CPA");
}

void testIdenticalCoordinates() {
    const auto result = evaluate(
        {0.0, 0.0, 10'000.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 10'000.0},
        {0.0, 0.0, 0.0});

    expect(result.conflict, "identical stationary aircraft conflict");
}

void testInvalidConfiguration() {
    bool negativeLookaheadRejected = false;
    bool nonFiniteThresholdRejected = false;

    try {
        static_cast<void>(ConflictDetector{DetectorConfig{.lookaheadSeconds = -1.0}});
    } catch (const std::invalid_argument&) {
        negativeLookaheadRejected = true;
    }

    try {
        static_cast<void>(ConflictDetector{DetectorConfig{
            .thresholds = {
                .horizontalMeters = std::numeric_limits<double>::infinity(),
                .verticalMeters = 150.0,
            },
        }});
    } catch (const std::invalid_argument&) {
        nonFiniteThresholdRejected = true;
    }

    expect(negativeLookaheadRejected, "negative lookahead rejected");
    expect(nonFiniteThresholdRejected, "non-finite threshold rejected");
}

} // namespace

int main() {
    failureCount += runCollisionDetectorTests();
    testHeadOnConflict();
    testNearMissConflict();
    testStaggeredCrossingIsSafe();
    testOverlappingDangerWindows();
    testSeparatedDangerWindows();
    testVerticalSeparationIsSafe();
    testVerticalConvergence();
    testDivergingAircraft();
    testConflictOutsideLookahead();
    testZeroRelativeVelocity();
    testStrictThresholdBoundary();
    testIdenticalCoordinates();
    testInvalidConfiguration();

    if (failureCount != 0) {
        std::cerr << failureCount << " detector test(s) failed.\n";
        return 1;
    }

    std::cout << "All conflict detector tests passed.\n";
    return 0;
}
