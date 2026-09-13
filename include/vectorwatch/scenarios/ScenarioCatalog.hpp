#pragma once

#include "vectorwatch/scenarios/Scenario.hpp"

#include <string>
#include <vector>

namespace vectorwatch {

class ScenarioCatalog {
public:
    static const std::vector<Scenario>& all() noexcept;
    static const Scenario* find(const std::string& name) noexcept;
};

} // namespace vectorwatch
