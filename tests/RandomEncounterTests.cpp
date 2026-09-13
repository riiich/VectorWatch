#include "RandomEncounterTests.hpp"

#include "vectorwatch/detection/CollisionDetector.hpp"
#include "vectorwatch/scenarios/RandomEncounterGenerator.hpp"

#include <cmath>
#include <cstdint>
#include <iostream>

namespace {

using vectorwatch::CollisionDetector;
using vectorwatch::RandomEncounter;
using vectorwatch::RandomEncounterGenerator;
using vectorwatch::RandomEncounterOutcome;
using vectorwatch::Vector3;

int failureCount = 0;

void expect(bool condition, const char* testName) {
    if (!condition) {
        std::cerr << "FAIL: " << testName << '\n';
        ++failureCount;
    }
}

bool equal(const Vector3& left, const Vector3& right) {
    return left.x == right.x && left.y == right.y && left.z == right.z;
}

double horizontalLength(const Vector3& vector) {
    return std::hypot(vector.x, vector.y);
}

double alongTrackWind(
    const Vector3& airVelocity,
    const Vector3& windVelocity) {
    const double airspeed = horizontalLength(airVelocity);
    return ((airVelocity.x * windVelocity.x) +
            (airVelocity.y * windVelocity.y)) /
        airspeed;
}

void verifyReasonableBounds(const RandomEncounter& encounter) {
    const double windSpeed = horizontalLength(encounter.windVelocity);
    expect(
        windSpeed >= 20.0 && windSpeed <= 35.0,
        "random wind speed stays within configured bounds");

    const Vector3 airVelocityA =
        encounter.aircraftA.velocity() - encounter.windVelocity;
    const Vector3 airVelocityB =
        encounter.aircraftB.velocity() - encounter.windVelocity;
    expect(
        horizontalLength(airVelocityA) >= 190.0 &&
            horizontalLength(airVelocityA) <= 260.0,
        "random aircraft A airspeed is reasonable");
    expect(
        horizontalLength(airVelocityB) >= 190.0 &&
            horizontalLength(airVelocityB) <= 260.0,
        "random aircraft B airspeed is reasonable");
    expect(
        std::abs(airVelocityA.z) <= 8.0 && std::abs(airVelocityB.z) <= 8.0,
        "random vertical rates are reasonable");
    expect(
        encounter.aircraftA.position().x < encounter.aircraftB.position().x,
        "aircraft A starts to the left of aircraft B");
    expect(
        airVelocityA.x > 0.0 && airVelocityB.x < 0.0,
        "aircraft fly toward each other from opposite sides");
    expect(
        std::abs(horizontalLength(airVelocityA) -
                 horizontalLength(airVelocityB)) >= 2.0 &&
            std::abs(horizontalLength(airVelocityA) -
                     horizontalLength(airVelocityB)) <= 10.0,
        "aircraft have distinct but comparable horizontal airspeeds");
    expect(
        std::isfinite(alongTrackWind(airVelocityA, encounter.windVelocity)) &&
            std::isfinite(alongTrackWind(airVelocityB, encounter.windVelocity)),
        "wind effect on each aircraft is finite");
}

void verifyPathReachesWaypoint(
    const vectorwatch::Aircraft& aircraft,
    const Vector3& waypoint,
    const char* testName) {
    const Vector3 displacement = waypoint - aircraft.position();
    const double arrivalTime =
        displacement.dot(aircraft.velocity()) /
        aircraft.velocity().lengthSquared();
    const Vector3 arrivalPosition =
        aircraft.position() + (aircraft.velocity() * arrivalTime);
    expect(arrivalTime > 0.0, testName);
    expect(
        std::sqrt((arrivalPosition - waypoint).lengthSquared()) <= 1.0e-9,
        testName);
}

} // namespace

int runRandomEncounterTests() {
    const RandomEncounter first = RandomEncounterGenerator::generate(42U);
    const RandomEncounter repeated = RandomEncounterGenerator::generate(42U);
    expect(equal(first.aircraftA.position(), repeated.aircraftA.position()),
        "random encounter seed reproduces aircraft A position");
    expect(equal(first.aircraftA.velocity(), repeated.aircraftA.velocity()),
        "random encounter seed reproduces aircraft A velocity");
    expect(equal(first.aircraftB.position(), repeated.aircraftB.position()),
        "random encounter seed reproduces aircraft B position");
    expect(equal(first.aircraftB.velocity(), repeated.aircraftB.velocity()),
        "random encounter seed reproduces aircraft B velocity");
    expect(equal(first.windVelocity, repeated.windVelocity),
        "random encounter seed reproduces wind");
    expect(equal(first.waypoint, repeated.waypoint),
        "random encounter seed reproduces shared waypoint");

    bool foundCollision = false;
    bool foundNonCollision = false;
    bool foundEastwardWind = false;
    bool foundWestwardWind = false;
    bool foundNorthwardWind = false;
    bool foundSouthwardWind = false;
    const CollisionDetector collisionDetector{};
    for (std::uint32_t seed = 1; seed <= 100; ++seed) {
        const RandomEncounter encounter = RandomEncounterGenerator::generate(seed);
        verifyReasonableBounds(encounter);
        verifyPathReachesWaypoint(
            encounter.aircraftA,
            encounter.waypoint,
            "aircraft A path reaches shared waypoint");
        verifyPathReachesWaypoint(
            encounter.aircraftB,
            encounter.waypoint,
            "aircraft B path reaches shared waypoint");
        const bool collides = collisionDetector
            .predict(encounter.aircraftA, encounter.aircraftB, 60.0)
            .hasValue();
        expect(
            collides == encounter.expectedCollision,
            "generated outcome matches nominal collision geometry");
        foundCollision = foundCollision || collides;
        foundNonCollision = foundNonCollision || !collides;
        foundEastwardWind =
            foundEastwardWind || encounter.windVelocity.x > 0.0;
        foundWestwardWind =
            foundWestwardWind || encounter.windVelocity.x < 0.0;
        foundNorthwardWind =
            foundNorthwardWind || encounter.windVelocity.y > 0.0;
        foundSouthwardWind =
            foundSouthwardWind || encounter.windVelocity.y < 0.0;
    }
    expect(foundCollision, "random generator produces collision encounters");
    expect(foundNonCollision, "random generator produces non-collision encounters");
    expect(
        foundEastwardWind && foundWestwardWind &&
            foundNorthwardWind && foundSouthwardWind,
        "random generator produces varied wind directions");

    const RandomEncounter forcedCollision = RandomEncounterGenerator::generate(
        12U,
        RandomEncounterOutcome::Collision);
    const RandomEncounter forcedPass = RandomEncounterGenerator::generate(
        12U,
        RandomEncounterOutcome::Pass);
    expect(
        collisionDetector
            .predict(
                forcedCollision.aircraftA,
                forcedCollision.aircraftB,
                60.0)
            .hasValue(),
        "forced collision encounter collides");
    expect(
        !collisionDetector
             .predict(forcedPass.aircraftA, forcedPass.aircraftB, 60.0)
             .hasValue(),
        "forced pass encounter does not collide");

    if (failureCount != 0) {
        std::cerr << failureCount << " random encounter test(s) failed.\n";
    }
    return failureCount;
}
