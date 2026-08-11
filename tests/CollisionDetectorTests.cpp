#include "CollisionDetectorTests.hpp"

#include "vectorwatch/detection/CollisionDetector.hpp"

#include <cmath>
#include <iostream>
#include <string_view>

namespace {

using vectorwatch::Aircraft;
using vectorwatch::CollisionDetector;
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

} // namespace

int runCollisionDetectorTests() {
    const CollisionDetector detector{};
    const Aircraft headOnA{
        1,
        Vector3{-10'000.0, 0.0, 10'000.0},
        Vector3{200.0, 0.0, 0.0}};
    const Aircraft headOnB{
        2,
        Vector3{10'000.0, 0.0, 10'000.0},
        Vector3{-200.0, 0.0, 0.0}};

    const auto collision = detector.predict(headOnA, headOnB, 60.0);
    expect(collision.has_value(), "head-on collision detected across time step");
    if (collision.has_value()) {
        expectNear(
            collision->timeSeconds,
            49.875,
            1.0e-9,
            "head-on collision time");
        expectNear(collision->position.x, 0.0, 1.0e-9, "head-on collision x");
        expectNear(
            collision->position.z,
            10'000.0,
            1.0e-9,
            "head-on collision altitude");
    }

    expect(
        !detector.predict(headOnA, headOnB, 40.0).has_value(),
        "collision beyond time step is ignored");

    const Aircraft nearMissB{
        2,
        Vector3{10'000.0, 600.0, 10'080.0},
        Vector3{-200.0, 0.0, 0.0}};
    expect(
        !detector.predict(headOnA, nearMissB, 60.0).has_value(),
        "separation conflict is not treated as physical collision");

    const Aircraft crossingA{
        1,
        Vector3{-10'000.0, 0.0, 10'000.0},
        Vector3{200.0, 0.0, 0.0}};
    const Aircraft crossingB{
        2,
        Vector3{0.0, -10'000.0, 10'000.0},
        Vector3{0.0, 200.0, 0.0}};
    expect(
        detector.predict(crossingA, crossingB, 60.0).has_value(),
        "crossing collision detected");

    if (failureCount != 0) {
        std::cerr << failureCount << " collision detector test(s) failed.\n";
    }
    return failureCount;
}
