#pragma once

#include "vectorwatch/scenarios/Scenario.hpp"

#include <span>
#include <string_view>

namespace vectorwatch {

class ScenarioCatalog {
public:
    [[nodiscard]] static std::span<const Scenario> all() noexcept;
    [[nodiscard]] static const Scenario* find(std::string_view name) noexcept;
};

} // namespace vectorwatch
