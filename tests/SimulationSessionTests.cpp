#include "SimulationSessionTests.hpp"

#include "vectorwatch/simulation/SimulationSession.hpp"

#include <cmath>
#include <iostream>

namespace {

using vectorwatch::Aircraft;
using vectorwatch::SimulationEndReason;
using vectorwatch::SimulationSession;
using vectorwatch::Vector3;

int failureCount = 0;

void expect(bool condition, const char* testName) {
    if (!condition) {
        std::cerr << "FAIL: " << testName << '\n';
        ++failureCount;
    }
}

void expectNear(
    double actual,
    double expected,
    double tolerance,
    const char* testName) {
    expect(std::abs(actual - expected) <= tolerance, testName);
}

SimulationSession headOnSession(double durationSeconds = 60.0) {
    return SimulationSession{
        Aircraft{1, Vector3{-10'000.0, 0.0, 10'000.0}, Vector3{200.0, 0.0, 0.0}},
        Aircraft{2, Vector3{10'000.0, 0.0, 10'000.0}, Vector3{-200.0, 0.0, 0.0}},
        durationSeconds};
}

} // namespace

int runSimulationSessionTests() {
    {
        SimulationSession session{
            Aircraft{1, Vector3{0.0, 0.0, 10'000.0}, Vector3{}},
            Aircraft{2, Vector3{50.0, 0.0, 10'000.0}, Vector3{}},
            60.0};
        expect(session.result().hasValue(), "initial contact completes session");
        expect(
            session.result()->reason == SimulationEndReason::Collision,
            "initial contact reports collision");
        expectNear(
            session.result()->simulationTimeSeconds,
            0.0,
            1.0e-9,
            "initial contact occurs at time zero");
    }

    {
        SimulationSession session{
            Aircraft{1, Vector3{0.0, 0.0, 10'000.0}, Vector3{20.0, 0.0, 0.0}},
            Aircraft{2, Vector3{0.0, 5.0, 10'500.0}, Vector3{20.0, 0.0, 0.0}},
            100.0,
            vectorwatch::WorldBounds(-10.0, 10.0, -10.0, 10.0)};
        session.advance(1.0);
        expect(
            session.result().hasValue() &&
                session.result()->reason == SimulationEndReason::Completed,
            "session completes after both aircraft leave world bounds");
        expectNear(
            session.snapshot().simulationTimeSeconds,
            1.0,
            1.0e-9,
            "world-bound completion precedes duration ceiling");
    }

    {
        SimulationSession session = headOnSession();
        session.advance(60.0);
        expect(session.result().hasValue(), "swept collision completes session");
        expect(
            session.result()->reason == SimulationEndReason::Collision,
            "swept collision reports collision");
        expectNear(
            session.snapshot().simulationTimeSeconds,
            49.875,
            1.0e-9,
            "session stops at first collision contact");
        expect(
            session.snapshot().sequence == 1,
            "collision advance increments sequence");
    }

    {
        SimulationSession session{
            Aircraft{1, Vector3{0.0, 0.0, 10'000.0}, Vector3{10.0, 0.0, 0.0}},
            Aircraft{2, Vector3{0.0, 5'000.0, 10'000.0}, Vector3{10.0, 0.0, 0.0}},
            2.0};
        session.advance(5.0);
        expect(session.result().hasValue(), "duration completes session");
        expect(
            session.result()->reason == SimulationEndReason::Completed,
            "duration reports normal completion");
        expectNear(
            session.snapshot().simulationTimeSeconds,
            2.0,
            1.0e-9,
            "advance clamps to remaining duration");
        expectNear(
            session.snapshot().aircraftA.position().x,
            20.0,
            1.0e-9,
            "completion snapshot contains final aircraft position");
    }

    {
        SimulationSession session = headOnSession();
        session.cancel();
        expect(
            session.result()->reason == SimulationEndReason::Cancelled,
            "cancel reports cancelled outcome");
        session.advance(10.0);
        expectNear(
            session.snapshot().simulationTimeSeconds,
            0.0,
            1.0e-9,
            "completed session ignores later advances");
    }

    if (failureCount != 0) {
        std::cerr << failureCount << " simulation session test(s) failed.\n";
    }
    return failureCount;
}
