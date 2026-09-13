#include "vectorwatch/scenarios/RandomEncounterGenerator.hpp"

#include <cmath>
#include <random>

namespace vectorwatch {
namespace {

constexpr double pi = 3.14159265358979323846;

double randomReal(
    std::mt19937& engine,
    double minimum,
    double maximum) {
    return std::uniform_real_distribution<double>{minimum, maximum}(engine);
}

bool chooseCollisionCourse(
    std::mt19937& engine,
    RandomEncounterOutcome outcome) {
    if (outcome == RandomEncounterOutcome::Collision) {
        return true;
    }
    if (outcome == RandomEncounterOutcome::Pass) {
        return false;
    }
    return std::bernoulli_distribution{0.4}(engine);
}

} // namespace

RandomEncounter RandomEncounterGenerator::generate(
    Optional<std::uint32_t> seed,
    RandomEncounterOutcome outcome) {
    const std::uint32_t actualSeed =
        seed.valueOr(static_cast<std::uint32_t>(std::random_device{}()));
    std::mt19937 engine{actualSeed};

    const double windSpeed = randomReal(engine, 20.0, 35.0);
    const double windDirection = randomReal(
        engine,
        0.0,
        2.0 * pi);
    const Vector3 windVelocity{
        windSpeed * std::cos(windDirection),
        windSpeed * std::sin(windDirection),
        0.0,
    };

    const double headingA = randomReal(
        engine,
        -pi / 9.0,
        pi / 9.0);
    const double crossingSide =
        std::bernoulli_distribution{0.5}(engine) ? 1.0 : -1.0;
    const double headingB = crossingSide * randomReal(
        engine,
        2.0 * pi / 3.0,
        5.0 * pi / 6.0);
    const double baseAirspeed = randomReal(engine, 195.0, 255.0);
    const double airspeedDifference = randomReal(engine, 1.0, 5.0) *
        (std::bernoulli_distribution{0.5}(engine) ? 1.0 : -1.0);
    const double airspeedA = baseAirspeed + airspeedDifference;
    const double airspeedB = baseAirspeed - airspeedDifference;
    const Vector3 airVelocityA{
        airspeedA * std::cos(headingA),
        airspeedA * std::sin(headingA),
        randomReal(engine, -8.0, 8.0),
    };
    const Vector3 airVelocityB{
        airspeedB * std::cos(headingB),
        airspeedB * std::sin(headingB),
        randomReal(engine, -8.0, 8.0),
    };
    const Vector3 groundVelocityA = airVelocityA + windVelocity;
    const Vector3 groundVelocityB = airVelocityB + windVelocity;
    const bool collisionCourse = chooseCollisionCourse(engine, outcome);
    const double arrivalTimeA = randomReal(engine, 20.0, 40.0);
    const double arrivalTimeB = collisionCourse
        ? arrivalTimeA
        : arrivalTimeA + randomReal(engine, 7.0, 12.0) *
              (std::bernoulli_distribution{0.5}(engine) ? 1.0 : -1.0);
    const Vector3 waypoint{
        randomReal(engine, -2'000.0, 2'000.0),
        randomReal(engine, -2'000.0, 2'000.0),
        randomReal(engine, 9'500.0, 10'500.0),
    };

    const Vector3 positionA = waypoint - (groundVelocityA * arrivalTimeA);
    const Vector3 positionB = waypoint - (groundVelocityB * arrivalTimeB);

    return RandomEncounter(
        Aircraft{1, positionA, groundVelocityA},
        Aircraft{2, positionB, groundVelocityB},
        waypoint,
        windVelocity,
        actualSeed,
        collisionCourse);
}

} // namespace vectorwatch
