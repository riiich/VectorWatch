#include "vectorwatch/bridge/VectorWatchBridge.h"
#include "vectorwatch/scenarios/ScenarioCatalog.hpp"
#include "vectorwatch/scenarios/ScenarioSetup.hpp"
#include "vectorwatch/simulation/ThreadedSimulationPipeline.hpp"
#include "vectorwatch/simulation/RadarBounds.hpp"
#include <cmath>
#include <cstring>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
using namespace vectorwatch;
namespace {
char* copy(const std::string& text) {
    auto* result = new char[text.size() + 1];
    std::memcpy(result, text.c_str(), text.size() + 1);
    return result;
}
void errorText(char** error) noexcept {
    if (!error) return;
    try { throw; }
    catch (const std::exception& e) { try { *error = copy(e.what()); } catch (...) {} }
    catch (...) { try { *error = copy("Native engine failure"); } catch (...) {} }
}
std::string quoted(const std::string& value) {
    std::ostringstream out;
    out << '"';
    for (unsigned char c : value) {
        if (c == '"' || c == '\\') out << '\\' << c;
        else if (c < 32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c);
        else out << c;
    }
    out << '"';
    return out.str();
}
void vector(std::ostream& out, const Vector3& v) {
    out << "{\"x\":" << v.x << ",\"y\":" << v.y << ",\"z\":" << v.z << '}';
}
void aircraft(std::ostream& out, const Aircraft& a) {
    out << "{\"id\":" << a.id() << ",\"position\":";
    vector(out, a.position()); out << ",\"velocity\":"; vector(out, a.velocity());
    out << ",\"speedMetersPerSecond\":" << a.velocity().length() << '}';
}
void collision(std::ostream& out, const Optional<Collision>& c) {
    if (!c.hasValue()) { out << "null"; return; }
    out << "{\"timeSeconds\":" << c->timeSeconds << ",\"position\":";
    vector(out, c->position); out << '}';
}
void window(std::ostream& out, const TimeWindow& w) {
    out << "{\"exists\":" << w.exists << ",\"startSeconds\":" << w.startSeconds
        << ",\"endSeconds\":" << w.endSeconds << '}';
}
struct Run {
    ScenarioSetup setup;
    ThreadedSimulationConfig config;
    WorldBounds radarBounds;
    ThreadedSimulationPipeline pipeline;
    bool stopRequested{};
    Run(ScenarioSetup s, ThreadedSimulationConfig c)
        : setup(std::move(s)), config(c),
          radarBounds(radarBoundsFor(setup.aircraftA, setup.aircraftB, config.durationSeconds, config.worldBounds)),
          pipeline(setup.aircraftA, setup.aircraftB, config) {}
};
}
extern "C" {
char* vw_scenarios(char** error) {
    if (error) *error = nullptr;
    try {
        std::ostringstream out; out << '[';
        for (const auto& s : ScenarioCatalog::all()) {
            out << "{\"name\":" << quoted(s.name) << ",\"description\":" << quoted(s.description) << "},";
        }
        out << "{\"name\":\"random-encounter\",\"description\":\"Seeded aircraft motion and wind.\"}]";
        return copy(out.str());
    } catch (...) { errorText(error); return nullptr; }
}
void* vw_create(const vw_options* o, char** error) {
    if (error) *error = nullptr;
    try {
        if (!o || !o->scenario || !std::isfinite(o->speed) || o->speed <= 0 || o->speed > 1000000 ||
            o->samples < 1 || o->samples > 100000 || o->workers < 0 || o->workers > 16 ||
            o->uncertainty < 0 || o->uncertainty > 2 || o->outcome < 0 || o->outcome > 2 ||
            (o->probabilistic != 0 && o->probabilistic != 1) || (o->has_seed != 0 && o->has_seed != 1))
            throw std::invalid_argument("Invalid run options");
        auto setup = prepareScenario(o->scenario,
            o->has_seed ? Optional<std::uint32_t>(o->seed) : Optional<std::uint32_t>(),
            static_cast<RandomEncounterOutcome>(o->outcome));
        ThreadedSimulationConfig config;
        config.durationSeconds = setup.durationSeconds;
        config.speedMultiplier = o->speed;
        config.windVelocity = setup.windVelocity;
        config.worldBounds = setup.worldBounds;
        config.prediction.mode = o->probabilistic ? PredictionMode::Probabilistic : PredictionMode::Deterministic;
        config.prediction.sampleCount = static_cast<std::size_t>(o->samples);
        config.prediction.workerCount = static_cast<std::size_t>(o->workers);
        config.prediction.uncertaintySeed = o->uncertainty_seed;
        config.prediction.uncertainty = uncertaintyConfigFor(static_cast<UncertaintyProfile>(o->uncertainty));
        return new Run(std::move(setup), config);
    } catch (...) { errorText(error); return nullptr; }
}
char* vw_read(void* handle, char** error) {
    if (error) *error = nullptr;
    try {
        if (!handle) throw std::invalid_argument("Missing run");
        const auto& run = *static_cast<Run*>(handle);
        const auto view = run.pipeline.view();
        std::ostringstream out; out.imbue(std::locale::classic()); out << std::setprecision(17) << std::boolalpha;
        out << "{\"seed\":";
        if (run.setup.seed.hasValue()) out << *run.setup.seed; else out << "null";
        out << ",\"durationSeconds\":" << run.config.durationSeconds << ",\"lookaheadSeconds\":" << run.config.prediction.detector.lookaheadSeconds;
        out << ",\"wind\":"; vector(out, run.setup.windVelocity);
        out << ",\"radarBounds\":{\"minimumX\":" << run.radarBounds.minimumX
            << ",\"maximumX\":" << run.radarBounds.maximumX
            << ",\"minimumY\":" << run.radarBounds.minimumY
            << ",\"maximumY\":" << run.radarBounds.maximumY << '}';
        out << ",\"waypoint\":";
        if (run.setup.waypoint.hasValue()) vector(out, *run.setup.waypoint); else out << "null";
        out << ",\"initialAircraft\":["; aircraft(out, run.setup.aircraftA); out << ','; aircraft(out, run.setup.aircraftB); out << ']';
        out << ",\"snapshot\":";
        if (!view.snapshot.hasValue()) out << "null";
        else {
            const auto& s = *view.snapshot;
            out << "{\"sequence\":" << s.sequence << ",\"simulationTimeSeconds\":" << s.simulationTimeSeconds << ",\"aircraft\":[";
            aircraft(out, s.aircraftA); out << ','; aircraft(out, s.aircraftB); out << "]}";
        }
        out << ",\"prediction\":";
        if (!view.prediction.hasValue()) out << "null";
        else {
            const auto& p = *view.prediction; const auto& a = p.nominalApproach;
            out << "{\"snapshotSequence\":" << p.snapshotSequence << ",\"simulationTimeSeconds\":" << p.simulationTimeSeconds
                << ",\"sampleCount\":" << p.sampleCount << ",\"conflictCount\":" << p.conflictCount << ",\"collisionCount\":" << p.collisionCount
                << ",\"conflictProbability\":" << p.conflictProbability << ",\"collisionProbability\":" << p.collisionProbability
                << ",\"riskLevel\":" << quoted(riskLevelName(p.riskLevel)) << ",\"workerCount\":" << p.workerCount
                << ",\"calculationMilliseconds\":" << p.calculationMilliseconds << ",\"nominalCollision\":";
            collision(out, p.nominalCollision);
            out << ",\"approach\":{\"currentDistanceMeters\":" << a.currentDistanceMeters << ",\"timeSeconds\":" << a.timeSeconds
                << ",\"horizontalSeparationMeters\":" << a.horizontalSeparationMeters << ",\"verticalSeparationMeters\":" << a.verticalSeparationMeters
                << ",\"minimumHorizontalSeparationMeters\":" << a.minimumHorizontalSeparationMeters << ",\"minimumHorizontalTimeSeconds\":" << a.minimumHorizontalTimeSeconds
                << ",\"minimumVerticalSeparationMeters\":" << a.minimumVerticalSeparationMeters << ",\"minimumVerticalTimeSeconds\":" << a.minimumVerticalTimeSeconds
                << ",\"hasRelativeMotion\":" << a.hasRelativeMotion << ",\"isWithinLookahead\":" << a.isWithinLookahead << ",\"conflict\":" << a.conflict;
            out << ",\"horizontalViolationWindow\":"; window(out, a.horizontalViolationWindow);
            out << ",\"verticalViolationWindow\":"; window(out, a.verticalViolationWindow);
            out << ",\"conflictWindow\":"; window(out, a.conflictWindow); out << "},\"paths\":[";
            // Rendering geometry uses the prediction's own snapshot time, never a newer snapshot.
            const Aircraft initial[] = {run.setup.aircraftA, run.setup.aircraftB};
            for (int i = 0; i < 2; ++i) {
                if (i) out << ',';
                auto projected = initial[i]; projected.update(p.simulationTimeSeconds);
                out << "{\"start\":"; vector(out, projected.position());
                auto cpa = projected; cpa.update(a.timeSeconds);
                projected.update(run.config.prediction.detector.lookaheadSeconds);
                out << ",\"end\":"; vector(out, projected.position());
                out << ",\"cpa\":"; vector(out, cpa.position()); out << '}';
            }
            out << "]}";
        }
        const bool final = view.result.hasValue() && (run.stopRequested || view.result->reason != SimulationEndReason::Completed ||
            (view.prediction.hasValue() && view.snapshot.hasValue() && view.prediction->snapshotSequence == view.snapshot->sequence));
        out << ",\"finished\":" << final << ",\"result\":";
        if (!view.result.hasValue()) out << "null";
        else {
            out << "{\"reason\":" << quoted(view.result->reason == SimulationEndReason::Collision ? "collision" :
                view.result->reason == SimulationEndReason::Completed ? "completed" : "stopped")
                << ",\"simulationTimeSeconds\":" << view.result->simulationTimeSeconds << ",\"collision\":";
            collision(out, view.result->collision); out << '}';
        }
        out << '}'; return copy(out.str());
    } catch (...) { errorText(error); return nullptr; }
}
void vw_stop(void* run) {
    if (run) {
        auto& value = *static_cast<Run*>(run);
        value.stopRequested = true;
        value.pipeline.requestStop();
    }
}
void vw_destroy(void* run) { delete static_cast<Run*>(run); }
void vw_free_string(char* text) { delete[] text; }
}
