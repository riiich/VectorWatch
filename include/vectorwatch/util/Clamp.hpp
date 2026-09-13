#pragma once

#include <algorithm>

namespace vectorwatch {

template <typename T>
T clampValue(const T& value, const T& minimum, const T& maximum) {
    return std::min(maximum, std::max(minimum, value));
}

} // namespace vectorwatch
