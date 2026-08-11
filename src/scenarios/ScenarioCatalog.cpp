#include "vectorwatch/scenarios/ScenarioCatalog.hpp"

#include <array>

namespace vectorwatch {
namespace {

const std::array scenarios{
    Scenario{
        "head-on",
        "Aircraft converge along the same horizontal line.",
        Aircraft{1, Vector3{-10'000.0, 0.0, 10'000.0}, Vector3{200.0, 0.0, 0.0}},
        Aircraft{2, Vector3{10'000.0, 0.0, 10'000.0}, Vector3{-200.0, 0.0, 0.0}},
        true,
    },
    Scenario{
        "parallel",
        "Aircraft maintain equal velocity and constant separation.",
        Aircraft{1, Vector3{0.0, 0.0, 10'000.0}, Vector3{200.0, 0.0, 0.0}},
        Aircraft{2, Vector3{0.0, 5'000.0, 10'000.0}, Vector3{200.0, 0.0, 0.0}},
        false,
    },
    Scenario{
        "crossing",
        "Aircraft reach the same intersection at the same time.",
        Aircraft{1, Vector3{-10'000.0, 0.0, 10'000.0}, Vector3{200.0, 0.0, 0.0}},
        Aircraft{2, Vector3{0.0, -10'000.0, 10'000.0}, Vector3{0.0, 200.0, 0.0}},
        true,
    },
    Scenario{
        "different-altitudes",
        "Horizontal paths cross, but vertical separation remains safe.",
        Aircraft{1, Vector3{-10'000.0, 0.0, 9'000.0}, Vector3{200.0, 0.0, 0.0}},
        Aircraft{2, Vector3{0.0, -10'000.0, 12'000.0}, Vector3{0.0, 200.0, 0.0}},
        false,
    },
    Scenario{
        "near-miss",
        "Aircraft pass without colliding but breach both separation thresholds.",
        Aircraft{1, Vector3{-10'000.0, 0.0, 10'000.0}, Vector3{200.0, 0.0, 0.0}},
        Aircraft{2, Vector3{10'000.0, 600.0, 10'080.0}, Vector3{-200.0, 0.0, 0.0}},
        true,
    },
    Scenario{
        "staggered-crossing",
        "Aircraft cross the same point at different times and remain safely separated.",
        Aircraft{1, Vector3{-10'000.0, 0.0, 10'000.0}, Vector3{200.0, 0.0, 0.0}},
        Aircraft{2, Vector3{0.0, -16'000.0, 10'000.0}, Vector3{0.0, 200.0, 0.0}},
        false,
    },
    Scenario{
        "offset-convergence",
        "Horizontal and vertical danger windows overlap away from the 3D CPA.",
        Aircraft{1, Vector3{0.0, 0.0, 10'000.0}, Vector3{0.0, 0.0, 0.0}},
        Aircraft{2, Vector3{2'000.0, 0.0, 10'700.0}, Vector3{-200.0, 0.0, -50.0}},
        true,
    },
    Scenario{
        "vertical-convergence",
        "Aircraft maintain horizontal proximity while converging in altitude.",
        Aircraft{1, Vector3{0.0, 0.0, 10'000.0}, Vector3{200.0, 0.0, 10.0}},
        Aircraft{2, Vector3{500.0, 0.0, 10'700.0}, Vector3{200.0, 0.0, -40.0}},
        true,
    },
    Scenario{
        "diverging",
        "Aircraft are moving apart and have no future loss of separation.",
        Aircraft{1, Vector3{-1'000.0, 0.0, 10'000.0}, Vector3{-200.0, 0.0, 0.0}},
        Aircraft{2, Vector3{1'000.0, 0.0, 10'000.0}, Vector3{200.0, 0.0, 0.0}},
        false,
    },
    Scenario{
        "outside-lookahead",
        "Aircraft converge, but not within the configured prediction horizon.",
        Aircraft{1, Vector3{-30'000.0, 0.0, 10'000.0}, Vector3{100.0, 0.0, 0.0}},
        Aircraft{2, Vector3{30'000.0, 0.0, 10'000.0}, Vector3{-100.0, 0.0, 0.0}},
        false,
    },
};

} // namespace

std::span<const Scenario> ScenarioCatalog::all() noexcept {
    return scenarios;
}

const Scenario* ScenarioCatalog::find(std::string_view name) noexcept {
    for (const auto& scenario : scenarios) {
        if (scenario.name == name) {
            return &scenario;
        }
    }
    return nullptr;
}

} // namespace vectorwatch
