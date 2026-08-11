#pragma once

#include "vectorwatch/model/Aircraft.hpp"

#include <string_view>

namespace vectorwatch {

struct Scenario {
    std::string_view name;
    std::string_view description;
    Aircraft aircraftA;
    Aircraft aircraftB;
    bool expectedConflict;
};

} // namespace vectorwatch
